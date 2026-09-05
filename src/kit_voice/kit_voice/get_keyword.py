import json
import os
import time

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from openai import RateLimitError
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

from kit_interfaces.srv import GetCommand

from kit_voice.wakeup_word import WakeupWord
from kit_voice.stt import STT

WAKEWORD_TIMEOUT = 30.0

PACKAGE_NAME = "kit_voice"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# class_names.json 은 kit_vision 쪽 산출물이 단일 진실 원천이다 (02-interfaces.md §3.1).
# name 검증 책임이 음성 노드에 있으므로 같은 파일을 그대로 읽어 프롬프트/검증에 쓴다.
CLASS_NAMES_PATH = os.path.join(
    get_package_share_directory("kit_vision"), "resource", "class_names.json"
)


def _load_class_names():
    with open(CLASS_NAMES_PATH, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return sorted(set(raw.values()))


def _build_prompt_template(class_names):
    names = ", ".join(class_names)
    content = f"""
        당신은 재난 구조키트 조립 명령에서 품목과 수량을 추출해야 합니다.

        <목표>
        - 문장에서 아래 품목 리스트에 있는 것만 최대한 정확히 추출하세요.
        - 각 품목의 수량도 함께 추출하세요. 수량 언급이 없으면 1로 간주합니다.
        - 어떤 재난 대비 키트인지(kit_type)도 문맥에서 유추하세요. 모르면 "unknown".

        <품목 리스트>
        {names}

        <출력 형식>
        - 아래 JSON 하나만 출력하세요. 다른 텍스트는 절대 출력하지 마세요.
        - {{{{"kit_type": "<키트 종류, 예: earthquake>", "items": [{{{{"name": "<품목 리스트 중 하나>", "qty": <1 이상 정수>}}}}]}}}}
        - 품목 리스트에 없는 물건은 절대 포함하지 마세요.

        <예시>
        - 입력: "지진 키트로 컵라면 두 개랑 마스크 하나 담아줘"
        출력: {{{{"kit_type": "earthquake", "items": [{{{{"name": "cup_ramen", "qty": 2}}}}, {{{{"name": "mask", "qty": 1}}}}]}}}}

        <사용자 입력>
        "{{user_input}}"
    """
    return PromptTemplate(input_variables=["user_input"], template=content)


def parse_and_validate(raw_response, class_names):
    """LLM 응답(JSON 텍스트)을 파싱하고 품목·수량을 검증한다 (02-interfaces.md §3.1 계약).

    반환: {"kit_type": str, "items": [{"name": str, "qty": int}, ...]}
    스키마 위반·미지원 품목·잘못된 수량이면 ValueError.
    """
    text = raw_response.strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.startswith("json"):
            text = text[4:]
        text = text.strip()

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f"LLM 응답이 JSON 이 아님: {e}") from e

    items = []
    for item in parsed.get("items", []):
        name = item.get("name")
        qty = item.get("qty")
        if name not in class_names:
            raise ValueError(f"미지원 품목: {name!r}")
        if not isinstance(qty, int) or qty < 1:
            raise ValueError(f"잘못된 수량: {name!r} qty={qty!r}")
        items.append({"name": name, "qty": qty})

    if not items:
        raise ValueError("추출된 품목이 없음")

    return {"kit_type": parsed.get("kit_type", "unknown"), "items": items}


class GetCommandNode(Node):
    def __init__(self):
        super().__init__("get_command_node")

        self.class_names = _load_class_names()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0.3, openai_api_key=OPENAI_API_KEY)
        self.prompt_template = _build_prompt_template(self.class_names)
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=OPENAI_API_KEY)
        self.wakeup_word = WakeupWord()

        self.get_command_srv = self.create_service(GetCommand, "/get_command", self.get_command)
        self.get_logger().info("GetCommandNode initialized. wait for client's request...")

    def _openai_error_code(self, e):
        # 크레딧 소진은 type=insufficient_quota / code=credit_balance_exhausted 로 온다.
        # 순수 rate limit 과 달리 기다려도 안 풀리므로 안내 문구를 구분한다.
        tag = f"{getattr(e, 'type', '')} {getattr(e, 'code', '')}"
        if "insufficient_quota" in tag or "credit_balance_exhausted" in tag:
            self.get_logger().error(
                "OpenAI 크레딧 소진 — 충전 필요: "
                "https://platform.openai.com/settings/organization/billing"
            )
            return "openai_quota_exhausted"
        self.get_logger().error(f"OpenAI rate limit — 잠시 후 재시도: {e}")
        return "openai_rate_limit"

    def get_command(self, request, response):
        # task_id 는 controller 가 작업 시작 시 생성해 요청에 담아 보낸다 (kit_interfaces
        # GetCommand.srv). 여기서는 그대로 받아 파싱 결과에 심어 로그·DB 기록에 흘려보낸다.
        # /kit/command_result 발행자가 붙으면 message.task_id = request.task_id 로 넘긴다.
        task_id = request.task_id

        try:
            self.wakeup_word.open()
        except Exception as e:
            self.get_logger().error(f"Error: Failed to open audio stream: {e}")
            response.success = False
            response.command_json = ""
            response.error_code = "stt_failed"
            return response

        detected = False
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < WAKEWORD_TIMEOUT:
                if self.wakeup_word.is_wakeup():
                    detected = True
                    break
        finally:
            self.wakeup_word.close()

        if not detected:
            self.get_logger().warn("wakeword timeout — no detection, returning failure")
            response.success = False
            response.command_json = ""
            response.error_code = "wakeword_timeout"
            return response

        # OpenAI 호출 실패를 콜백 밖으로 흘리면 노드가 통째로 죽는다.
        # 실패는 서비스 실패로 돌려주고 노드는 살려 둔다.
        try:
            raw_text = self.stt.speech2text()
        except RateLimitError as e:
            response.success = False
            response.command_json = ""
            response.error_code = self._openai_error_code(e)
            return response
        except Exception as e:
            self.get_logger().error(f"STT 실패: {type(e).__name__}: {e}")
            response.success = False
            response.command_json = ""
            response.error_code = "stt_failed"
            return response

        try:
            llm_output = self.lang_chain.invoke({"user_input": raw_text}).content
            command = parse_and_validate(llm_output, self.class_names)
        except ValueError as e:
            self.get_logger().warn(f"명령 검증 실패: {e}")
            response.success = False
            response.command_json = ""
            response.error_code = "invalid_command"
            return response
        except RateLimitError as e:
            response.success = False
            response.command_json = ""
            response.error_code = self._openai_error_code(e)
            return response
        except Exception as e:
            self.get_logger().error(f"OpenAI 호출 실패: {type(e).__name__}: {e}")
            response.success = False
            response.command_json = ""
            response.error_code = "openai_error"
            return response

        command["raw_text"] = raw_text
        command["task_id"] = task_id
        self.get_logger().info(f"command: {command}")

        response.success = True
        response.command_json = json.dumps(command, ensure_ascii=False)
        response.error_code = ""
        return response


def _demo():
    # OpenAI/마이크 없이도 도는 순수 파싱·검증 self-check.
    class_names = {"cup_ramen", "mask"}

    ok = parse_and_validate(
        '{"kit_type": "earthquake", "items": '
        '[{"name": "cup_ramen", "qty": 2}, {"name": "mask", "qty": 1}]}',
        class_names,
    )
    assert ok == {
        "kit_type": "earthquake",
        "items": [{"name": "cup_ramen", "qty": 2}, {"name": "mask", "qty": 1}],
    }, ok

    # 코드블록으로 감싼 응답도 파싱돼야 한다.
    fenced = parse_and_validate(
        '```json\n{"kit_type": "earthquake", "items": [{"name": "mask", "qty": 1}]}\n```',
        class_names,
    )
    assert fenced["items"] == [{"name": "mask", "qty": 1}], fenced

    for bad in (
        '{"items": [{"name": "hammer", "qty": 1}]}',       # 미지원 품목
        '{"items": [{"name": "mask", "qty": 0}]}',          # 잘못된 수량
        '{"items": [{"name": "mask", "qty": "1"}]}',        # qty 타입 오류
        '{"items": []}',                                     # 품목 없음
        'not json',                                          # 파싱 실패
    ):
        try:
            parse_and_validate(bad, class_names)
            assert False, f"통과하면 안 되는 입력이 통과함: {bad}"
        except ValueError:
            pass

    print("ok")


def main():
    rclpy.init()
    node = GetCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    _demo()
