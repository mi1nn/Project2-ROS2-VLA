# Controller.py
# 상태 전이·서비스 요청·결과 발행을 담당하며 실제 로봇 API는 Motion에 위임한다.
import json
import math
import time
from datetime import datetime, timezone
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from builtin_interfaces.msg import Time

from kit_interfaces.msg import TaskStatus, ComponentResult
from kit_interfaces.srv import (
    GetCommand,
    GetComponentPose,
    InspectKit,
)
from kit_robot.controller_model import (
    Attempt,
    validate_command,
    build_components,
)

# 데모 실행용 의존성: 실제 운용 시 main의 주입 객체를 구현된 Motion으로 교체한다.
from kit_robot.motion_demo import MotionDemo

class State(Enum):
    '''명령 수신부터 최종 보고까지 Controller의 일곱 실행 상태를 정의한다.'''
    IDLE = auto()
    LISTEN = auto()
    VALIDATE = auto()
    OBSERVE = auto()
    EXECUTE = auto()
    INSPECT = auto()
    REPORT = auto()


class Controller(Node):
    '''Motion과 비동기 서비스를 연결해 Component 순차 실행과 결과 발행을 관리한다.'''
    def __init__(self, motion=None):
        '''Motion을 주입받고 상태 처리기, ROS 파라미터, 서비스·토픽 및 timer를 구성한다.'''
        super().__init__("controller")

        # Motion 의존성과 상태 진입 표시: entered=True인 tick에서만 진입 작업을 수행한다.
        self.motion = motion
        self.state = State.IDLE
        self.state_entered = True

        # 상태별 처리기 매핑: timer는 이 표에서 현재 상태의 메서드를 선택한다.
        self.handlers = {
            State.IDLE: self.handle_idle,
            State.LISTEN: self.handle_listen,
            State.VALIDATE: self.handle_validate,
            State.OBSERVE: self.handle_observe,
            State.EXECUTE: self.handle_execute,
            State.INSPECT: self.handle_inspect,
            State.REPORT: self.handle_report,
        }

        # 명령 client: task_id를 전달하고 음성 노드의 검증된 명령 JSON을 받는다.
        self.command_client = self.create_client(
            GetCommand, "/get_command"
        )

        # 준비 대기와 응답 대기는 별개. 초 단위 기본값이며 YAML로 조정한다.
        # 명령 응답 시간은 웨이크워드 대기와 STT·LLM 처리 시간을 합쳐 실측한다.
        self.declare_parameter("service_ready_timeout_sec", 20.0)   # 서버 준비 제한: 20초
        self.declare_parameter("command_timeout_sec", 60.0)         # 명령 전체 응답 제한: 60초

        self.service_ready_timeout = self.get_parameter(
            "service_ready_timeout_sec"
        ).value
        self.command_timeout = self.get_parameter(
            "command_timeout_sec"
        ).value

        if self.service_ready_timeout <= 0 or self.command_timeout <= 0:
            raise ValueError("서비스 timeout은 양수여야 합니다.")

        # YAML의 지원 품목과 공용 슬롯 이름. 품목은 class_names.json과 일치시킨다.
        # 슬롯 목록 순서가 배치 순서이며 실제 좌표 조회는 Motion이 담당한다.
        self.declare_parameter(
            "supported_names", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            "slot_names", Parameter.Type.STRING_ARRAY
        )

        self.supported_names = set(
            self.get_parameter("supported_names").value or []
        )
        self.slot_names = list(
            self.get_parameter("slot_names").value or []
        )

        if not self.supported_names or not self.slot_names:
            raise ValueError("지원 품목과 슬롯 목록을 설정해야 합니다.")

        # 좌표 client: 관찰 시 TCP 자세를 보내 베이스 좌표계 파지 자세를 받는다.
        self.pose_client = self.create_client(
            GetComponentPose, "/get_component_pose"
        )

        # 좌표 응답 제한 5초는 실측 전 기본값이다. 검출 나이와 정착 시간도 초 단위다.
        # 초기값 1.0/1.2초는 동일 시계 기준을 전제로 이동 전 프레임 사용을 줄인다.
        # 정착 시간을 검출 허용 나이보다 길게 유지하고 시계 오차·촬영 지연을 검증한다.
        self.declare_parameter("pose_timeout_sec", 5.0)
        self.declare_parameter("max_age_sec", 1.0)
        self.declare_parameter("observation_settle_sec", 1.2)

        self.pose_timeout = self.get_parameter(
            "pose_timeout_sec"
        ).value
        self.max_age_sec = self.get_parameter(
            "max_age_sec"
        ).value
        self.observation_settle = self.get_parameter(
            "observation_settle_sec"
        ).value

        if self.pose_timeout <= 0 or self.max_age_sec <= 0:
            raise ValueError("좌표 timeout과 검출 허용 나이는 양수여야 합니다.")

        if self.observation_settle <= self.max_age_sec:
            raise ValueError("관찰 정착 시간은 검출 허용 나이보다 길어야 합니다.")

        # 최초 시도를 포함한 상한: 2이면 최초 1회 + 재시도 1회다.
        self.declare_parameter("max_attempts", 2)
        self.max_attempts = self.get_parameter("max_attempts").value

        if type(self.max_attempts) is not int or self.max_attempts < 1:
            raise ValueError("max_attempts는 1 이상 정수여야 합니다.")

        # 검사 client: 배치 성공 수가 아닌 원래 기대 수량과 실물 구성을 비교한다.
        self.inspect_client = self.create_client(
            InspectKit, "/inspect_kit"
        )

        # 검사 응답 제한과 검사 자세 정착 시간. 관찰과 별도로 실측해 조정한다.
        self.declare_parameter("inspect_timeout_sec", 5.0)
        self.declare_parameter("inspection_settle_sec", 1.2)

        self.inspect_timeout = self.get_parameter(
            "inspect_timeout_sec"
        ).value
        self.inspection_settle = self.get_parameter(
            "inspection_settle_sec"
        ).value

        if self.inspect_timeout <= 0:
            raise ValueError("검사 timeout은 양수여야 합니다.")

        if self.inspection_settle <= self.max_age_sec:
            raise ValueError("검사 정착 시간은 검출 허용 나이보다 길어야 합니다.")

        # 상태 전이·최종 결과 토픽: 기본 RELIABLE, depth=10의 발행 큐를 사용한다.
        self.task_status_pub = self.create_publisher(
            TaskStatus, "/kit/task_status", 10
        )

        # 품목 최종 결과 토픽: 재시도 내역은 Attempt 배열로 한 메시지에 담는다.
        self.component_result_pub = self.create_publisher(
            ComponentResult, "/kit/component_result", 10
        )

        # 최종 발행 이후 다음 작업까지의 간격. 복귀 동작의 대기 시간이 아니다.
        # API 제한 등으로 즉시 반복하지 않도록 운영 조건에 맞춰 조정한다.
        self.declare_parameter("restart_delay_sec", 5.0)
        self.restart_delay = self.get_parameter("restart_delay_sec").value

        if not math.isfinite(self.restart_delay) or self.restart_delay <= 0:
            raise ValueError("재시작 간격은 유한한 양수여야 합니다.")

        # 상태 확인 주기 0.1초. 동기 Motion 호출 중에는 이 주기가 보장되지 않는다.
        self.timer = self.create_timer(0.1, self.timer_tick)


    def transition_to(self, next_state: State):
        '''실패 발생 상태를 보존하고 다음 상태의 첫 진입 표시와 RUNNING 발행을 처리한다.'''
        self.get_logger().info(
            f"{self.state.name} -> {next_state.name}"
        )
        if next_state == State.REPORT and self.task_fatal:
            self.failure_stage = self.state.name

        self.state = next_state
        self.state_entered = True

        if next_state != State.IDLE:
            self.publish_task_status()


    def timer_tick(self):
        '''진입 플래그를 소비한 뒤 현재 상태의 처리기를 한 번 호출한다.'''
        # 먼저 소비해야 handler 내부 전환으로 설정한 True가 유지됩니다.
        entered = self.state_entered
        self.state_entered = False
        self.handlers[self.state](entered)


    def handle_idle(self, entered: bool):
        '''새 작업 ID와 실행 변수를 초기화하고 명령 대기 상태로 전환한다.'''
        if not entered:
            return

        # 작업 식별·이력은 UTC 시각, timeout·정착은 monotonic 시각으로 구분한다.
        # 단일 Controller 기준으로 UTC 마이크로초를 포함한 작업 ID를 생성한다.
        now = datetime.now(timezone.utc)
        self.task_id = f"TASK-{now:%Y%m%dT%H%M%S%fZ}"

        # 새 작업의 명령·실행 목록. expected_counts는 실행 실패가 있어도 줄이지 않는다.
        self.command_json = ""
        self.kit_type = ""
        self.components = []
        self.expected_counts = {}
        self.component_index = 0        # 실행 목록에서 처리할 위치
        self.target_pose = None

        # 동시에 하나의 서비스 future만 관리한다. deadline과 준비 시각은 monotonic 초다.
        self.pending_future = None      # 서비스 응답을 기다릴 객체
        self.request_deadline = None
        self.service_ready_deadline = None
        self.pose_ready_at = None

        # 검사 미수행은 None, 수행 결과는 PASS/FAIL/ERROR 객체로 구분한다.
        # task_fatal은 현재 작업 종료 여부이며 새 작업 재시작 허용 여부와는 별개다.
        self.inspection_result = None
        self.task_fatal = False
        self.error_code = ""
        self.detail = ""

        # 이동 시작 여부로 복귀 필요성을 판단하고 REPORT는 작업당 한 번 완료한다.
        self.motion_started = False
        self.restart_allowed = True     # 명령 timeout 등에서 자동 재시작 방지
        self.report_completed = False   # REPORT의 중복 처리 방지
        self.failure_stage = ""

        # 현재 작업에서 발행한 index 집합. DB 저장 완료를 확인하는 장치는 아니다.
        self.published_component_indices = set()

        self.get_logger().info(f"작업 시작: {self.task_id}")
        self.transition_to(State.LISTEN)


    def handle_listen(self, entered: bool):
        '''명령 서비스 준비를 기다려 한 번 요청하고 이후 tick에서 응답을 확인한다.'''
        now = time.monotonic()

        if entered:
            self.service_ready_deadline = (
                now + self.service_ready_timeout
            )

        # 이미 요청한 상태에서 재전송 방지 + 요청과 응답 확인 연결
        if self.pending_future is not None:
            self.check_command_response()
            return

        if now >= self.service_ready_deadline:
            self.task_fatal = True
            self.error_code = "command_service_unavailable"
            self.detail = "/get_command 서비스 준비 시간 초과"
            self.restart_allowed = False
            self.transition_to(State.REPORT)
            return

        # 서비스 준비 여부 확인 -> 반환해서 다음 tick 대기
        if not self.command_client.service_is_ready():
            return

        # 요청 생성
        request = GetCommand.Request()
        request.task_id = self.task_id

        try:
            # 요청을 비동기로 전송
            self.pending_future = self.command_client.call_async(
                request
            )
        except Exception as error:
            self.task_fatal = True
            self.error_code = "command_request_failed"
            self.detail = str(error)
            self.restart_allowed = False
            self.transition_to(State.REPORT)
            return

        self.request_deadline = (
            time.monotonic() + self.command_timeout
        )
        self.get_logger().info("명령 요청 전송")


    def handle_validate(self, entered: bool):
        '''명령을 검증하고 공용 슬롯을 배정한 뒤 Component와 기대 수량을 저장한다.'''
        if not entered:
            return

        try:
            command = validate_command(
                self.command_json,
                self.supported_names,
            )
            components, expected = build_components(
                command,
                self.slot_names,
            )

        except ValueError as error:
            self.fail_command(
                "invalid_command",
                str(error),
                restart_allowed=True,
            )
            return

        # 검증과 슬롯 배정이 모두 성공한 뒤 저장합니다.
        self.kit_type = command["kit_type"]
        self.components = components
        self.expected_counts = expected
        self.component_index = 0

        self.get_logger().info(
            f"명령 검증 완료: Component {len(components)}개"
        )
        self.transition_to(State.OBSERVE)


    def handle_observe(self, entered: bool):
        '''시도를 시작해 관찰 자세로 이동하고 정착 후 좌표 요청과 응답 확인을 진행한다.'''
        if entered:
            component = self.components[self.component_index]
            now = datetime.now(timezone.utc)

            if component.started_at is None:
                component.started_at = now

            component.attempts.append(
                Attempt(
                    attempt_no=component.attempt_count + 1,
                    started_at=now,
                )
            )

            # 새 시도에서는 이전 목표 좌표와 다른 상태에서 사용한 deadline을 폐기한다.
            self.target_pose = None
            self.service_ready_deadline = None
            self.request_deadline = None

            try:
                # 호출 도중 실패해도 복구 검토 대상이 되도록 먼저 표시
                self.motion_started = True
                self.motion.move_to_observation_pose()
            except Exception as error:
                self.task_fatal = True
                self.error_code = "observation_move_failed"
                self.detail = str(error)
                self.transition_to(State.REPORT)
                return

            self.pose_ready_at = (
                time.monotonic() + self.observation_settle
            )
            return

        if time.monotonic() < self.pose_ready_at:
            return

        # 진행 중인 요청은 응답만 확인하여 같은 좌표 요청을 중복 전송하지 않는다.
        if self.pending_future is not None:
            self.check_pose_response()
            return

        now = time.monotonic()

        # 정착이 끝난 이후부터 서비스 준비 제한 시간을 계산합니다.
        if self.service_ready_deadline is None:
            self.service_ready_deadline = (
                now + self.service_ready_timeout
            )

        if now >= self.service_ready_deadline:
            self.task_fatal = True
            self.error_code = "pose_service_unavailable"
            self.detail = "/get_component_pose 서비스 준비 시간 초과"
            self.transition_to(State.REPORT)
            return

        if not self.pose_client.service_is_ready():
            return

        try:
            robot_pose = self.motion.get_current_pose()
            self.request_component_pose(robot_pose)
        except Exception as error:
            self.task_fatal = True
            self.error_code = "pose_request_failed"
            self.detail = str(error)
            self.transition_to(State.REPORT)



    def handle_execute(self, entered: bool):
        '''현재 Component의 파지·배치를 한 번 수행하고 성공 또는 실패 경로로 전환한다.'''
        if not entered:
            return

        component = self.components[self.component_index]
        attempt = component.attempts[-1]
        # 같은 EXECUTE 안의 파지·배치 중 어느 단계에서 예외가 났는지 기록한다.
        stage = "PICK"

        try:
            picked = self.motion.pick_component(
                component.name,
                self.target_pose,
            )

            if not picked:
                self.handle_grasp_failure()
                return

            stage = "PLACE"
            self.motion.place_component(
                component.name,
                component.slot,
            )

        except Exception as error:
            now = datetime.now(timezone.utc)

            attempt.status = "FAILED"
            attempt.ended_at = now
            attempt.failed_stage = stage
            attempt.error_code = f"{stage.lower()}_failed"
            attempt.detail = str(error)

            component.status = "FAILED"
            component.ended_at = now
            component.error_code = attempt.error_code
            component.detail = attempt.detail

            self.target_pose = None
            self.task_fatal = True
            self.error_code = component.error_code
            self.detail = component.detail
            self.transition_to(State.REPORT)
            return

        now = datetime.now(timezone.utc)

        attempt.status = "SUCCESS"
        attempt.ended_at = now
        component.status = "SUCCESS"
        component.ended_at = now
        self.target_pose = None

        self.get_logger().info(
            f"배치 완료: {component.name} → {component.slot}"
        )

        self.publish_component_result(component)
        self.component_index += 1

        if self.component_index < len(self.components):
            self.transition_to(State.OBSERVE)
        else:
            self.transition_to(State.INSPECT)


    def handle_inspect(self, entered: bool):
        '''검사 자세 이동과 정착 후 실물 검사를 요청하고 응답을 확인한다.'''
        if entered:
            self.service_ready_deadline = None
            self.request_deadline = None

            try:
                self.motion_started = True
                self.motion.move_to_inspection_pose()
            except Exception as error:
                self.fail_inspection("inspection_move_failed", str(error))
                return

            self.pose_ready_at = (
                time.monotonic() + self.inspection_settle
            )
            return

        if time.monotonic() < self.pose_ready_at:
            return

        # 요청 이후에는 재전송 없이 응답 완료와 timeout만 확인한다.
        if self.pending_future is not None:
            self.check_inspection_response()
            return

        now = time.monotonic()

        if self.service_ready_deadline is None:
            self.service_ready_deadline = (
                now + self.service_ready_timeout
            )

        if now >= self.service_ready_deadline:
            self.fail_inspection(
                "inspection_service_unavailable",
                "/inspect_kit 서비스 준비 시간 초과",
            )
            return

        if not self.inspect_client.service_is_ready():
            return

        try:
            self.request_inspection()
        except Exception as error:
            self.fail_inspection("inspection_request_failed", str(error))


    def handle_report(self, entered: bool):
        '''미완료 결과와 복귀를 정리해 최종 상태를 발행하고 허용된 경우 재대기한다.'''
        if not self.report_completed:
            self.finalize_components()

            # 이미 안전 복구가 실패한 경우 자동으로 반복하지 않습니다.
            recovery_failed = self.error_code == "recovery_failed"

            if recovery_failed:
                self.restart_allowed = False

            elif self.motion_started:
                try:
                    if self.task_fatal:
                        self.motion.recover_to_safe_pose()
                    else:
                        self.motion.move_home()
                except Exception as error:
                    self.task_fatal = True
                    self.error_code = "recovery_failed"
                    self.detail = str(error)
                    self.restart_allowed = False

            # Component 실패 이력과 무관하게 실물 PASS와 치명 오류·복귀 결과로 판정한다.
            inspection_passed = (
                self.inspection_result is not None
                and self.inspection_result["result"] == "PASS"
            )

            final_status = (
                "SUCCESS"
                if inspection_passed and not self.task_fatal
                else "FAILED"
            )

            self.publish_task_status(final_status)
            self.get_logger().info(
                f"작업 종료: {self.task_id}, {final_status}"
            )

            # 최종 발행 이후부터 재시작 간격을 측정하며 복귀·발행을 반복하지 않는다.
            self.report_completed = True
            self.restart_ready_at = (
                time.monotonic() + self.restart_delay
            )
            return

        if not self.restart_allowed:
            return

        if time.monotonic() >= self.restart_ready_at:
            self.transition_to(State.IDLE)


    def check_command_response(self):
        '''명령 응답·timeout을 확인하고 성공 시 검증, 실패 시 종료 경로를 선택한다.'''
        future = self.pending_future

        # 1. 응답 대기 시간 초과 확인
        if not future.done():
            if time.monotonic() >= self.request_deadline:
                future.cancel()
                self.fail_command(
                    "command_timeout",
                    "명령 응답 대기 시간 초과",
                )
            return

        try:
            response = future.result()
            if response is None:
                raise ValueError("명령 응답이 없습니다.")
        except Exception as error:
            # 2. future에서 예외 발생 또는 응답 없음 -> 실패 처리
            self.fail_command("command_response_failed", str(error))
            return

        self.pending_future = None
        self.request_deadline = None

        if not response.success:
            # 서버가 실패 응답을 완료한 경우만 재대기한다. quota·미지 오류는 제외한다.
            retryable_codes = {
                "wakeword_timeout",
                "stt_failed",
                "invalid_command",
                "openai_rate_limit",
                "openai_error",
            }
            self.fail_command(
                response.error_code or "command_failed",
                "음성 노드가 명령 처리 실패를 반환했습니다.",
                restart_allowed=response.error_code in retryable_codes,
            )
            return

        self.command_json = response.command_json
        self.transition_to(State.VALIDATE)


    def fail_command(
        self,
        error_code: str,
        detail: str,
        restart_allowed: bool = False,
    ):
        '''명령 실패 정보를 저장하고 자동 재시작 허용 여부와 함께 REPORT로 전환한다.'''
        self.pending_future = None
        self.request_deadline = None
        self.task_fatal = True
        self.error_code = error_code
        self.detail = detail
        self.restart_allowed = restart_allowed

        self.get_logger().error(f"{error_code}: {detail}")
        self.transition_to(State.REPORT)


    def request_component_pose(self, robot_pose):
        '''현재 품목과 관찰 TCP 자세를 전송하고 좌표 응답 future와 제한 시각을 저장한다.'''
        if self.pending_future is not None:
            raise RuntimeError("이미 진행 중인 서비스 요청이 있습니다.")

        component = self.components[self.component_index]

        request = GetComponentPose.Request()
        request.component = component.name
        request.robot_posx = robot_pose
        request.max_age_sec = self.max_age_sec
        # 초기 정책: 프레임마다 바뀔 수 있는 픽셀 중심을 물체 ID처럼 누적하지 않는다.
        request.exclude_taken = []

        self.pending_future = self.pose_client.call_async(request)
        self.request_deadline = (
            time.monotonic() + self.pose_timeout
        )

        self.get_logger().info(
            f"좌표 요청: {component.name}, slot={component.slot}"
        )


    def check_pose_response(self):
        '''좌표 응답·timeout과 자세 형식을 확인해 실행 또는 오류 처리로 전환한다.'''
        future = self.pending_future

        if not future.done():
            if time.monotonic() >= self.request_deadline:
                future.cancel()
                self.fail_pose_request(
                    "pose_timeout",
                    "좌표 응답 대기 시간 초과",
                )
            return

        try:
            response = future.result()
            if response is None:
                raise ValueError("좌표 응답이 없습니다.")
        except Exception as error:
            self.fail_pose_request("pose_response_failed", str(error))
            return

        self.pending_future = None
        self.request_deadline = None

        if not response.success:
            self.handle_pose_failure(response.error_code or "pose_failed")
            return

        pose = list(response.target_pose)

        if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
            self.fail_pose_request(
                "invalid_target_pose",
                "목표 자세는 유한한 숫자 6개여야 합니다.",
            )
            return

        self.target_pose = pose
        self.transition_to(State.EXECUTE)


    def fail_pose_request(self, error_code: str, detail: str):
        '''좌표 통신·응답 오류를 작업 치명 오류로 기록하고 기존 좌표를 폐기한다.'''
        self.pending_future = None
        self.request_deadline = None
        self.target_pose = None

        self.task_fatal = True
        self.error_code = error_code
        self.detail = detail

        self.get_logger().error(f"{error_code}: {detail}")
        self.transition_to(State.REPORT)


    def handle_pose_failure(self, error_code: str):
        '''좌표 오류를 분류해 재관찰하거나 Component 실패를 확정하고 다음 품목으로 진행한다.'''
        # 다시 관찰할 오류와 즉시 해당 Component를 종료할 오류를 분리한다.
        retryable = {"stale", "not_detected"}
        component_fatal = {"no_candidate", "out_of_workspace"}

        if error_code not in retryable | component_fatal:
            self.fail_pose_request(error_code, "알 수 없는 좌표 오류")
            return

        component = self.components[self.component_index]
        attempt = component.attempts[-1]
        now = datetime.now(timezone.utc)

        attempt.status = "FAILED"
        attempt.ended_at = now
        attempt.failed_stage = "OBSERVE"
        attempt.error_code = error_code
        attempt.detail = "좌표 획득 실패"

        self.target_pose = None

        if (
            error_code in retryable
            and component.attempt_count < self.max_attempts
        ):
            self.get_logger().warning(
                f"{component.name}: {error_code}, 재관찰"
            )
            self.transition_to(State.OBSERVE)
            return

        component.status = "FAILED"
        component.ended_at = now
        component.error_code = (
            "max_attempts" if error_code in retryable else error_code
        )
        component.detail = f"마지막 좌표 오류: {error_code}"

        self.publish_component_result(component)
        self.component_index += 1

        if self.component_index < len(self.components):
            self.transition_to(State.OBSERVE)
        else:
            self.transition_to(State.INSPECT)


    def handle_grasp_failure(self):
        '''파지 실패를 기록하고 안전 복구 후 재시도하며 복구 실패 시 작업을 종료한다.'''
        component = self.components[self.component_index]
        attempt = component.attempts[-1]

        attempt.status = "FAILED"
        attempt.ended_at = datetime.now(timezone.utc)
        attempt.failed_stage = "PICK"
        attempt.error_code = "grasp_failed"
        attempt.detail = "파지 확인 실패"
        self.target_pose = None

        try:
            self.motion.recover_to_safe_pose()
        except Exception as error:
            component.status = "FAILED"
            component.ended_at = datetime.now(timezone.utc)
            component.error_code = "recovery_failed"
            component.detail = str(error)

            self.task_fatal = True
            self.error_code = component.error_code
            self.detail = component.detail
            self.restart_allowed = False
            self.transition_to(State.REPORT)
            return

        if component.attempt_count < self.max_attempts:
            self.get_logger().warning(
                f"{component.name}: 파지 실패, 복구 완료 후 재시도"
            )
            self.transition_to(State.OBSERVE)
            return

        component.status = "FAILED"
        component.ended_at = datetime.now(timezone.utc)
        component.error_code = "max_attempts"
        component.detail = "파지 실패로 최대 시도 횟수 도달"

        self.publish_component_result(component)
        self.component_index += 1

        if self.component_index < len(self.components):
            self.transition_to(State.OBSERVE)
        else:
            self.transition_to(State.INSPECT)


    def request_inspection(self):
        '''원래 명령의 품목·기대 수량으로 검사를 요청하고 응답 대기 정보를 저장한다.'''
        if self.pending_future is not None:
            raise RuntimeError("이미 진행 중인 서비스 요청이 있습니다.")

        request = InspectKit.Request()

        # 품목명과 수량 배열은 같은 순서를 유지하고 원래 요청 수량을 사용한다.
        names = list(self.expected_counts)
        request.expected_classes = names
        request.expected_counts = [
            self.expected_counts[name] for name in names
        ]
        request.max_age_sec = self.max_age_sec

        self.pending_future = self.inspect_client.call_async(request)
        self.request_deadline = (
            time.monotonic() + self.inspect_timeout
        )

        self.get_logger().info(
            f"검사 요청: {self.expected_counts}"
        )


    def fail_inspection(self, error_code: str, detail: str):
        '''검사 불능을 ERROR로 기록하고 작업 실패 보고로 전환한다.'''
        self.pending_future = None
        self.request_deadline = None

        # 검사 불능은 구성 불일치와 구분한다. 무효 수량·검출 나이는 JSON null로 남긴다.
        self.inspection_result = {
            "result": "ERROR",
            "expected_counts": dict(self.expected_counts),
            "actual_counts": None,
            "missing": [],
            "unexpected": [],
            "detection_age": None,
            "inspected_at": datetime.now(timezone.utc).isoformat(),
        }

        self.task_fatal = True
        self.error_code = error_code
        self.detail = detail

        self.get_logger().error(f"{error_code}: {detail}")
        self.transition_to(State.REPORT)


    def check_inspection_response(self):
        '''응답 최신성과 수량을 검증한 뒤 PASS·FAIL을 저장하고 오류는 ERROR로 처리한다.'''
        future = self.pending_future

        if not future.done():
            if time.monotonic() >= self.request_deadline:
                future.cancel()
                self.fail_inspection(
                    "inspection_timeout",
                    "검사 응답 대기 시간 초과",
                )
            return

        try:
            response = future.result()
            if response is None:
                raise ValueError("검사 응답이 없습니다.")

            age = response.detection_age
            counts = list(response.actual_counts)
            # 수량 배열은 원래 검사 요청의 품목 순서로 해석한다.
            names = list(self.expected_counts)

            if (
                not math.isfinite(age)
                or age < 0
                or age > self.max_age_sec
            ):
                raise ValueError("검출 데이터가 없거나 유효하지 않은 시각입니다.")

            if len(counts) != len(names) or any(v < 0 for v in counts):
                raise ValueError("검사 수량 응답이 올바르지 않습니다.")

        except Exception as error:
            self.fail_inspection("inspection_response_invalid", str(error))
            return

        self.pending_future = None
        self.request_deadline = None

        self.inspection_result = {
            "result": "PASS" if response.ok else "FAIL",
            "expected_counts": dict(self.expected_counts),
            "actual_counts": dict(zip(names, counts)),
            "missing": list(response.missing),
            "unexpected": list(response.unexpected),
            "detection_age": float(age),
            "inspected_at": datetime.now(timezone.utc).isoformat(),
        }

        if not response.ok:
            self.error_code = "inspection_mismatch"
            self.detail = "실제 키트 구성이 기대 구성과 다릅니다."

        self.get_logger().info(
            f"검사 결과: {self.inspection_result['result']}"
        )
        self.transition_to(State.REPORT)


    def publish_task_status(self, task_status="RUNNING"):
        '''현재 작업 상태와 검사 결과를 TaskStatus로 변환해 발행한다.'''
        message = TaskStatus()
        message.task_id = self.task_id
        message.state = self.state.name
        message.task_status = task_status
        message.kit_type = self.kit_type
        message.component_total = len(self.components)

        # 관찰·실행 중인 품목이 없으면 빈 이름과 -1로 표현한다.
        message.current_component = ""
        message.current_component_index = -1

        if (
            self.state in {State.OBSERVE, State.EXECUTE}
            and self.component_index < len(self.components)
        ):
            component = self.components[self.component_index]
            message.current_component = component.name
            message.current_component_index = component.index

        message.inspection_result = (
            json.dumps(
                self.inspection_result,
                ensure_ascii=False,
                allow_nan=False,
            )
            if self.inspection_result is not None
            else ""
        )
        message.error_code = self.error_code
        message.detail = self.detail
        message.stamp = self.get_clock().now().to_msg()

        self.task_status_pub.publish(message)


    @staticmethod
    def to_ros_time(value: datetime) -> Time:
        '''timezone이 있는 datetime을 ROS 메시지의 초·나노초 필드로 변환한다.'''
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("시각에는 timezone이 있어야 합니다.")

        message = Time()
        message.sec = int(value.timestamp())
        message.nanosec = value.microsecond * 1000
        return message


    def publish_component_result(self, component):
        '''종료된 Component와 Attempt 이력을 직렬화해 작업 내 중복 없이 발행한다.'''
        if component.index in self.published_component_indices:
            return

        # 모든 Attempt의 종료 시각이 채워진 최종 Component만 전달받는다.
        # 시각은 timezone 포함 ISO 문자열이며 SKIPPED의 이력은 빈 배열이다.
        attempts = []

        for attempt in component.attempts:
            attempts.append({
                "attempt_no": attempt.attempt_no,
                "status": attempt.status,
                "started_at": attempt.started_at.isoformat(),
                "ended_at": attempt.ended_at.isoformat(),
                "failed_stage": attempt.failed_stage,
                "error_code": attempt.error_code,
                "detail": attempt.detail,
            })

        message = ComponentResult()
        message.task_id = self.task_id
        message.component_index = component.index
        message.component_total = len(self.components)
        message.component = component.name
        message.slot = component.slot
        message.status = component.status
        message.attempt_count = component.attempt_count
        message.attempts_json = json.dumps(
            attempts,
            ensure_ascii=False,
            allow_nan=False,
        )
        message.error_code = component.error_code
        message.detail = component.detail
        message.started_at = self.to_ros_time(component.started_at)
        message.ended_at = self.to_ros_time(component.ended_at)

        self.component_result_pub.publish(message)

        # 발행 호출 후 index를 기록해 프로세스 내 중복 발행을 방지한다.
        self.published_component_indices.add(component.index)


    def finalize_components(self):
        '''종료 결과를 보존하고 진행 중 품목은 FAILED, 미시작 품목은 SKIPPED로 정리한다.'''
        now = datetime.now(timezone.utc)

        for component in self.components:
            # 이미 종료된 결과는 보존합니다.
            if component.status not in {"SUCCESS", "FAILED", "SKIPPED"}:
                if component.attempts:
                    component.status = "FAILED"
                    component.error_code = (
                        self.error_code or "task_aborted"
                    )
                    component.detail = self.detail

                    attempt = component.attempts[-1]

                    if attempt.ended_at is None:
                        attempt.status = "FAILED"
                        attempt.ended_at = now
                        attempt.error_code = component.error_code
                        attempt.detail = component.detail
                        attempt.failed_stage = self.failure_stage
                else:
                    # 실제 시작 시각이 없으므로 건너뛰기로 확정한 시각을 양쪽에 기록한다.
                    component.status = "SKIPPED"
                    component.started_at = now
                    component.error_code = "task_aborted"
                    component.detail = "작업 중단으로 실행하지 않음"

                component.ended_at = now

            self.publish_component_result(component)


def main(args=None):
    '''MotionDemo를 주입한 Controller를 실행하고 종료 시 ROS 자원을 정리한다.'''
    rclpy.init(args=args)
    node = Controller(motion=MotionDemo())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
