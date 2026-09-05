# controller_model.py
from dataclasses import dataclass, field
from datetime import datetime

import json

@dataclass
class Attempt:
    '''관찰부터 파지, 배치까지 시도 한 번의 기록'''
    attempt_no: int
    started_at: datetime
    ended_at: datetime | None = None
    status: str = "RUNNING"
    failed_stage: str = ""
    error_code: str = ""
    detail: str = ""


@dataclass
class Component:
    '''물체 하나의 실행 기록'''
    name: str
    index: int
    slot: str
    status: str = "PENDING"
    # default_factory : Component마다 별도의 시도 목록을 만든다.
    attempts: list[Attempt] = field(default_factory=list)
    error_code: str = ""
    detail: str = ""
    started_at: datetime | None = None
    ended_at: datetime | None = None

    @property
    def attempt_count(self) -> int:
        '''목록 길이를 계산해 횟수와 기록이 어긋나지 않도록 관리'''
        return len(self.attempts)


def validate_command(
    command_json: str,
    supported_names: set[str],
) -> dict:
    '''
    명령 JSON 검증 함수
    정상이면 검증된 명령을 반환, 잘못된 입력이면 ValueError 발생
    '''
    try:
        command = json.loads(command_json)
    except (TypeError, ValueError) as error:
        raise ValueError("올바른 JSON 명령이 아닙니다.") from error

    if not isinstance(command, dict):
        raise ValueError("명령은 JSON 객체여야 합니다.")

    if not isinstance(command.get("kit_type"), str):
        raise ValueError("kit_type은 문자열이어야 합니다.")

    items = command.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError("items는 비어 있지 않은 목록이어야 합니다.")

    validated_items = []

    for item in items:
        if not isinstance(item, dict):
            raise ValueError("각 품목은 JSON 객체여야 합니다.")

        name = item.get("name")
        qty = item.get("qty")

        if not isinstance(name, str) or name not in supported_names:
            raise ValueError(f"지원하지 않는 품목: {name!r}")

        # bool도 int의 하위 타입이므로 정확한 타입으로 검사합니다.
        if type(qty) is not int or qty < 1:
            raise ValueError(f"{name}: 수량은 1 이상 정수여야 합니다.")

        validated_items.append({"name": name, "qty": qty})

    return {
        "kit_type": command["kit_type"],
        "items": validated_items,
    }


def build_components(
    command: dict,
    slot_names: list[str],
) -> tuple[list[Component], dict[str, int]]:
    '''검증된 명령을 Component 목록으로 바꾸고 슬롯을 할당'''

    # 중복 품목 합산: 최초 등장 순서를 유지합니다.
    expected_counts: dict[str, int] = {}

    for item in command["items"]:
        name = item["name"]
        expected_counts[name] = (
            expected_counts.get(name, 0) + item["qty"]
        )

    if any(
        not isinstance(slot, str) or not slot.strip()
        for slot in slot_names
    ):
        raise ValueError("슬롯 이름은 비어 있지 않은 문자열이어야 합니다.")

    if len(slot_names) != len(set(slot_names)):
        raise ValueError("슬롯 이름이 중복되었습니다.")

    total = sum(expected_counts.values())

    if total > len(slot_names):
        raise ValueError(
            f"배치 공간 부족: 요청 {total}개, 슬롯 {len(slot_names)}개"
        )

    components = []

    for name, qty in expected_counts.items():
        for _ in range(qty):
            index = len(components)
            components.append(
                Component(
                    name=name,
                    index=index,
                    slot=slot_names[index],
                )
            )

    return components, expected_counts
