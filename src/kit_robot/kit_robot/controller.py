import time
from datetime import datetime, timezone
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from kit_interfaces.srv import GetCommand
from kit_robot.controller_model import (
    validate_command,
    build_components,
)


class State(Enum):
    '''상태 이름 정의'''
    IDLE = auto()
    LISTEN = auto()
    VALIDATE = auto()
    OBSERVE = auto()
    EXECUTE = auto()
    INSPECT = auto()
    REPORT = auto()


class Controller(Node):
    '''상태 머신'''
    def __init__(self, motion=None):
        super().__init__("controller")

        # 노드 초기화
        self.motion = motion
        self.state = State.IDLE
        self.state_entered = True

        self.handlers = {
            State.IDLE: self.handle_idle,
            State.LISTEN: self.handle_listen,
            State.VALIDATE: self.handle_validate,
            State.OBSERVE: self.handle_observe,
            State.EXECUTE: self.handle_execute,
            State.INSPECT: self.handle_inspect,
            State.REPORT: self.handle_report,
        }

        # command 노드에 명령을 요청하는 서비스 생성
        self.command_client = self.create_client(
            GetCommand, "/get_command"
        )

        # ==============================================
        # 실제 동작 시 값 조정 필요
        # ==============================================
        self.declare_parameter("service_ready_timeout_sec", 20.0)   # 서비스 호출 대기 시간
        self.declare_parameter("command_timeout_sec", 60.0)         # 요청 후 응답 대기 시간

        self.service_ready_timeout = self.get_parameter(
            "service_ready_timeout_sec"
        ).value
        self.command_timeout = self.get_parameter(
            "command_timeout_sec"
        ).value

        if self.service_ready_timeout <= 0 or self.command_timeout <= 0:
            raise ValueError("서비스 timeout은 양수여야 합니다.")

        # ==============================================
        # 설정 파일과 연결로 구현할 부분 - 품목, 슬롯 위치 등
        # 일차적으로 ROS 파라미터로 전달받는 방식으로 구현해둠.
        # ==============================================
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


        # 현재 상태 함수 실행
        self.timer = self.create_timer(0.1, self.timer_tick)


    def transition_to(self, next_state: State):
        self.get_logger().info(
            f"{self.state.name} -> {next_state.name}"
        )
        '''상태 변경 관리'''
        self.state = next_state
        self.state_entered = True


    def timer_tick(self):
        # 먼저 소비해야 handler 내부 전환으로 설정한 True가 유지됩니다.
        entered = self.state_entered
        self.state_entered = False
        self.handlers[self.state](entered)


    def handle_idle(self, entered: bool):
        '''IDLE 진입'''
        if not entered:
            return

        # task_id 생성 : TASK-생성시간
        now = datetime.now(timezone.utc)
        self.task_id = f"TASK-{now:%Y%m%dT%H%M%S%fZ}"

        # 명령과 실행 목록
        self.command_json = ""
        self.kit_type = ""
        self.components = []
        self.expected_counts = {}
        self.component_index = 0        # 실행 목록에서 처리할 위치
        self.target_pose = None

        # 비동기 서비스와 정착 대기
        self.pending_future = None      # 서비스 응답을 기다릴 객체
        self.request_deadline = None
        self.service_ready_deadline = None
        self.pose_ready_at = None

        # 작업 결과
        self.inspection_result = None
        self.task_fatal = False
        self.error_code = ""
        self.detail = ""

        # 종료 처리
        self.motion_started = False
        self.restart_allowed = True     # 명령 timeout 등에서 자동 재시작 방지
        self.report_completed = False   # REPORT의 중복 처리 방지

        self.get_logger().info(f"작업 시작: {self.task_id}")
        self.transition_to(State.LISTEN)


    def handle_listen(self, entered: bool):
        '''LISTEN 진입'''
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
        '''VALIDATION 진입'''
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
        pass


    def handle_execute(self, entered: bool):
        pass


    def handle_inspect(self, entered: bool):
        pass


    def handle_report(self, entered: bool):
        pass


    def check_command_response(self):
        '''명령 응답 상태 확인과 다음 상태 결정'''
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
        '''명령 실패 공통 처리'''
        self.pending_future = None
        self.request_deadline = None
        self.task_fatal = True
        self.error_code = error_code
        self.detail = detail
        self.restart_allowed = restart_allowed

        self.get_logger().error(f"{error_code}: {detail}")
        self.transition_to(State.REPORT)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()

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
