"""Coordinate task states, ROS services, motion, and result publishing."""

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
from kit_interfaces.srv import GetCommand, GetComponentPose, InspectKit
from kit_robot.controller_model import Attempt, build_components, validate_command

from kit_robot.motion import Motion


class State(Enum):
    IDLE = auto()
    LISTEN = auto()
    VALIDATE = auto()
    OBSERVE = auto()
    EXECUTE = auto()
    INSPECT = auto()
    REPORT = auto()


class TransitionCategory(Enum):
    NORMAL = "NORMAL"
    RETRY = "RETRY"
    COMPONENT_FATAL = "COMPONENT_FATAL"
    TASK_FATAL = "TASK_FATAL"


class Controller(Node):
    def __init__(self, motion=None):
        super().__init__("controller", namespace="/dsr01")

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

        self.command_client = self.create_client(GetCommand, "/get_command")

        self.declare_parameter("service_ready_timeout_sec", 20.0)
        self.declare_parameter("command_timeout_sec", 60.0)

        self.service_ready_timeout = self.get_parameter(
            "service_ready_timeout_sec"
        ).value
        self.command_timeout = self.get_parameter("command_timeout_sec").value

        if self.service_ready_timeout <= 0 or self.command_timeout <= 0:
            raise ValueError("서비스 timeout은 양수여야 합니다.")

        self.declare_parameter("supported_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("slot_names", Parameter.Type.STRING_ARRAY)

        self.supported_names = set(self.get_parameter("supported_names").value or [])
        self.slot_names = list(self.get_parameter("slot_names").value or [])

        if not self.supported_names or not self.slot_names:
            raise ValueError("지원 품목과 슬롯 목록을 설정해야 합니다.")

        self.pose_client = self.create_client(GetComponentPose, "/get_component_pose")

        self.declare_parameter("pose_timeout_sec", 5.0)
        self.declare_parameter("max_age_sec", 1.0)
        self.declare_parameter("observation_settle_sec", 1.2)

        self.pose_timeout = self.get_parameter("pose_timeout_sec").value
        self.max_age_sec = self.get_parameter("max_age_sec").value
        self.observation_settle = self.get_parameter("observation_settle_sec").value

        if self.pose_timeout <= 0 or self.max_age_sec <= 0:
            raise ValueError("좌표 timeout과 검출 허용 나이는 양수여야 합니다.")

        if self.observation_settle <= self.max_age_sec:
            raise ValueError("관찰 정착 시간은 검출 허용 나이보다 길어야 합니다.")

        # max_attempts includes the initial attempt.
        self.declare_parameter("max_attempts", 2)
        self.max_attempts = self.get_parameter("max_attempts").value

        if type(self.max_attempts) is not int or self.max_attempts < 1:
            raise ValueError("max_attempts는 1 이상 정수여야 합니다.")

        self.inspect_client = self.create_client(InspectKit, "/inspect_kit")

        self.declare_parameter("inspect_timeout_sec", 5.0)
        self.declare_parameter("inspection_settle_sec", 1.2)

        self.inspect_timeout = self.get_parameter("inspect_timeout_sec").value
        self.inspection_settle = self.get_parameter("inspection_settle_sec").value

        if self.inspect_timeout <= 0:
            raise ValueError("검사 timeout은 양수여야 합니다.")

        if self.inspection_settle <= self.max_age_sec:
            raise ValueError("검사 정착 시간은 검출 허용 나이보다 길어야 합니다.")

        self.task_status_pub = self.create_publisher(TaskStatus, "/kit/task_status", 10)
        self.component_result_pub = self.create_publisher(
            ComponentResult, "/kit/component_result", 10
        )

        self.declare_parameter("restart_delay_sec", 5.0)
        self.restart_delay = self.get_parameter("restart_delay_sec").value

        if not math.isfinite(self.restart_delay) or self.restart_delay <= 0:
            raise ValueError("재시작 간격은 유한한 양수여야 합니다.")

    def transition_to(
        self,
        next_state: State,
        category: TransitionCategory,
        reason: str,
    ):
        previous_state = self.state
        message = (
            f"[STATE] {previous_state.name} -> {next_state.name} | "
            f"category={category.value} | reason={reason}"
        )

        if category == TransitionCategory.NORMAL:
            self.get_logger().info(message)
        elif category in {
            TransitionCategory.RETRY,
            TransitionCategory.COMPONENT_FATAL,
        }:
            self.get_logger().warning(message)
        else:
            self.get_logger().error(message)

        if next_state == State.REPORT and self.task_fatal:
            self.failure_stage = previous_state.name

        self.state = next_state
        self.state_entered = True

        if next_state != State.IDLE:
            self.publish_task_status()

    def fail_task(
        self,
        error_code: str,
        detail: str,
        reason: str,
        *,
        restart_allowed=None,
    ):
        if self.pending_future is not None and not self.pending_future.done():
            self.pending_future.cancel()
        self.pending_future = None
        self.request_deadline = None
        self.target_pose = None
        self.task_fatal = True
        self.error_code = error_code
        self.detail = detail
        if restart_allowed is not None:
            self.restart_allowed = restart_allowed
        self.transition_to(State.REPORT, TransitionCategory.TASK_FATAL, reason)

    def advance_component(self, component, category, next_reason, final_reason):
        self.publish_component_result(component)
        self.component_index += 1
        next_state = (
            State.OBSERVE
            if self.component_index < len(self.components)
            else State.INSPECT
        )
        reason = next_reason if next_state == State.OBSERVE else final_reason
        self.transition_to(next_state, category, reason)

    def timer_tick(self):
        # Consume first so transitions made by the handler preserve the new True value.
        entered = self.state_entered
        self.state_entered = False
        self.handlers[self.state](entered)

    def handle_idle(self, entered: bool):
        if not entered:
            return

        now = datetime.now(timezone.utc)
        self.task_id = f"TASK-{now:%Y%m%dT%H%M%S%fZ}"

        self.command_json = ""
        self.kit_type = ""
        self.components = []
        self.expected_counts = {}
        self.component_index = 0
        self.target_pose = None

        # Only one service request may be pending at a time.
        self.pending_future = None
        self.request_deadline = None
        self.service_ready_deadline = None
        self.pose_ready_at = None

        self.inspection_result = None
        self.task_fatal = False
        self.error_code = ""
        self.detail = ""

        self.motion_started = False
        self.restart_allowed = True
        self.report_completed = False
        self.failure_stage = ""

        # Motion owns the static OctoMap so it can be reused across tasks.
        self.place_octomap_ready = bool(
            self.motion is not None
            and getattr(
                self.motion,
                "static_octomap_initialized",
                False,
            )
        )
        # Set only while the initial OctoMap scan is active.
        self.place_octomap_settle_at = None

        # Prevent duplicate component results when REPORT finalizes the task.
        self.published_component_indices = set()

        self.transition_to(
            State.LISTEN,
            TransitionCategory.NORMAL,
            "새 작업 초기화 완료",
        )

    def handle_listen(self, entered: bool):
        now = time.monotonic()

        if entered:
            self.service_ready_deadline = now + self.service_ready_timeout

        if self.pending_future is not None:
            self.check_command_response()
            return

        if now >= self.service_ready_deadline:
            code = "command_service_unavailable"
            detail = "/get_command 서비스 준비 시간 초과"
            self.fail_task(
                code,
                detail,
                f"명령 서비스 준비 실패: {code}, {detail}",
                restart_allowed=False,
            )
            return

        if not self.command_client.service_is_ready():
            return

        request = GetCommand.Request()
        request.task_id = self.task_id

        try:
            self.pending_future = self.command_client.call_async(request)
        except Exception as error:
            code = "command_request_failed"
            detail = str(error)
            self.fail_task(
                code,
                detail,
                f"명령 요청 전송 실패: {code}, {detail}",
                restart_allowed=False,
            )
            return

        self.request_deadline = time.monotonic() + self.command_timeout
        self.get_logger().info("명령 요청 전송")

    def handle_validate(self, entered: bool):
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

        self.kit_type = command["kit_type"]
        self.components = components
        self.expected_counts = expected
        self.component_index = 0

        self.transition_to(
            State.OBSERVE,
            TransitionCategory.NORMAL,
            f"명령 검증 완료, Component {len(components)}개 실행 시작",
        )

    def _enter_observation_pose(self):
        try:
            self.motion_started = True
            # Object exclusion is applied after the target pose or object cloud is known.
            self.motion.move_to_observation_pose()
        except Exception as error:
            code = "observation_move_failed"
            detail = str(error)
            self.fail_task(
                code,
                detail,
                f"관찰 자세 이동 실패: {code}, {detail}",
            )
            return

        self.pose_ready_at = time.monotonic() + self.observation_settle

    def handle_observe(self, entered: bool):
        """Start an attempt, settle at observation pose, and request perception."""
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

            self.target_pose = None
            self.service_ready_deadline = None
            self.request_deadline = None

            if not self.place_octomap_ready:
                if getattr(
                    self.motion,
                    "static_octomap_initialized",
                    False,
                ):
                    self.place_octomap_ready = True
                    self._enter_observation_pose()
                    return

                try:
                    self.motion_started = True
                    scan_started = self.motion.start_static_octomap_scan()
                except Exception as error:
                    code = "initial_octomap_scan_failed"
                    detail = str(error)
                    self.fail_task(
                        code,
                        detail,
                        f"초기 고정 OctoMap 스캔 시작 실패: {code}, {detail}",
                    )
                    return

                if not scan_started:
                    self.place_octomap_ready = True
                    self._enter_observation_pose()
                    return

                self.place_octomap_settle_at = time.monotonic() + float(
                    self.motion.static_octomap_scan_sec
                )
                return

            self._enter_observation_pose()
            return

        if self.place_octomap_settle_at is not None:
            if time.monotonic() < self.place_octomap_settle_at:
                return

            try:
                self.motion.finish_static_octomap_scan()
            except Exception as error:
                code = "initial_octomap_scan_failed"
                detail = str(error)
                self.fail_task(
                    code,
                    detail,
                    f"초기 고정 OctoMap 확정 실패: {code}, {detail}",
                )
                return

            self.place_octomap_ready = True
            self.place_octomap_settle_at = None
            self._enter_observation_pose()
            return

        if time.monotonic() < self.pose_ready_at:
            return

        # Cup noodles run live 6D pose and grasp generation after settling.
        component = self.components[self.component_index]
        if component.name == self.motion.cup_grasp_component:
            self.target_pose = None
            self.transition_to(
                State.EXECUTE,
                TransitionCategory.NORMAL,
                "컵라면 관찰 자세 정착 완료; live GraspGenX 실행 시작",
            )
            return

        if self.pending_future is not None:
            self.check_pose_response()
            return

        now = time.monotonic()

        if self.service_ready_deadline is None:
            self.service_ready_deadline = now + self.service_ready_timeout

        if now >= self.service_ready_deadline:
            code = "pose_service_unavailable"
            detail = "/get_component_pose 서비스 준비 시간 초과"
            self.fail_task(
                code,
                detail,
                f"좌표 서비스 준비 실패: {code}, {detail}",
            )
            return

        if not self.pose_client.service_is_ready():
            return

        try:
            robot_pose = self.motion.get_current_pose()
            self.request_component_pose(robot_pose)
        except Exception as error:
            code = "pose_request_failed"
            detail = str(error)
            self.fail_task(
                code,
                detail,
                f"좌표 요청 전송 실패: {code}, {detail}",
            )

    def handle_execute(self, entered: bool):
        if not entered:
            return

        component = self.components[self.component_index]
        attempt = component.attempts[-1]
        stage = "PICK"

        try:
            if component.name == self.motion.cup_grasp_component:
                self.get_logger().info(
                    "컵라면 전용 PICK: FoundationPose -> GraspGenX -> "
                    "MoveIt 후보 검증"
                )
                picked = self.motion.pick_cup_graspgenx_live(component.name)
            else:
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

            self.fail_task(
                component.error_code,
                component.detail,
                (
                    f"{component.name} {stage} 실행 실패: "
                    f"{component.error_code}, {component.detail}"
                ),
            )
            return

        now = datetime.now(timezone.utc)

        attempt.status = "SUCCESS"
        attempt.ended_at = now
        component.status = "SUCCESS"
        component.ended_at = now
        self.target_pose = None

        self.advance_component(
            component,
            TransitionCategory.NORMAL,
            f"{component.name} 배치 완료, 다음 Component 관찰 시작",
            f"마지막 Component {component.name} 배치 완료",
        )

    def handle_inspect(self, entered: bool):
        if entered:
            self.service_ready_deadline = None
            self.request_deadline = None

            try:
                self.motion_started = True
                self.motion.move_to_inspection_pose()
            except Exception as error:
                self.fail_inspection("inspection_move_failed", str(error))
                return

            self.pose_ready_at = time.monotonic() + self.inspection_settle
            return

        if time.monotonic() < self.pose_ready_at:
            return

        if self.pending_future is not None:
            self.check_inspection_response()
            return

        now = time.monotonic()

        if self.service_ready_deadline is None:
            self.service_ready_deadline = now + self.service_ready_timeout

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
        """Finalize results, recover after fatal errors, and schedule the next task."""
        if not self.report_completed:
            self.finalize_components()

            recovery_failed = self.error_code == "recovery_failed"

            if recovery_failed:
                self.restart_allowed = False

            elif self.motion_started:
                try:
                    if self.task_fatal:
                        self.motion.recover_to_safe_pose()
                except Exception as error:
                    self.task_fatal = True
                    self.error_code = "recovery_failed"
                    self.detail = str(error)
                    self.restart_allowed = False

            inspection_passed = (
                self.inspection_result is not None
                and self.inspection_result["result"] == "PASS"
            )

            final_status = (
                "SUCCESS" if inspection_passed and not self.task_fatal else "FAILED"
            )

            self.publish_task_status(final_status)
            self.get_logger().info(f"작업 종료: {self.task_id}, {final_status}")

            self.report_completed = True
            self.restart_ready_at = time.monotonic() + self.restart_delay
            return

        if not self.restart_allowed:
            return

        if time.monotonic() >= self.restart_ready_at:
            self.transition_to(
                State.IDLE,
                TransitionCategory.NORMAL,
                "재시작 대기 시간 경과",
            )

    def check_command_response(self):
        future = self.pending_future

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
            self.fail_command("command_response_failed", str(error))
            return

        self.pending_future = None
        self.request_deadline = None

        if not response.success:
            # Only known recoverable voice errors may start a new task automatically.
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
        self.transition_to(
            State.VALIDATE,
            TransitionCategory.NORMAL,
            "명령 서비스 응답 수신 성공",
        )

    def fail_command(
        self,
        error_code: str,
        detail: str,
        restart_allowed: bool = False,
    ):
        self.fail_task(
            error_code,
            detail,
            f"명령 처리 실패: {error_code}, {detail}",
            restart_allowed=restart_allowed,
        )

    def request_component_pose(self, robot_pose):
        if self.pending_future is not None:
            raise RuntimeError("이미 진행 중인 서비스 요청이 있습니다.")

        component = self.components[self.component_index]

        request = GetComponentPose.Request()
        request.component = component.name
        request.robot_posx = robot_pose
        request.max_age_sec = self.max_age_sec
        # Pixel centers are not stable object IDs across frames.
        request.exclude_taken = []

        self.pending_future = self.pose_client.call_async(request)
        self.request_deadline = time.monotonic() + self.pose_timeout

        self.get_logger().info(f"좌표 요청: {component.name}, slot={component.slot}")

    def check_pose_response(self):
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
        component = self.components[self.component_index]
        self.transition_to(
            State.EXECUTE,
            TransitionCategory.NORMAL,
            f"{component.name}의 유효한 목표 좌표 획득",
        )

    def fail_pose_request(self, error_code: str, detail: str):
        self.fail_task(
            error_code,
            detail,
            f"좌표 요청 처리 실패: {error_code}, {detail}",
        )

    def handle_pose_failure(self, error_code: str):
        """Retry transient perception failures and finalize permanent failures."""
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

        if error_code in retryable and component.attempt_count < self.max_attempts:
            self.transition_to(
                State.OBSERVE,
                TransitionCategory.RETRY,
                (
                    f"{component.name} 좌표 획득 실패: {error_code}, "
                    f"next_attempt={component.attempt_count + 1}/"
                    f"{self.max_attempts}"
                ),
            )
            return

        component.status = "FAILED"
        component.ended_at = now
        component.error_code = "max_attempts" if error_code in retryable else error_code
        component.detail = f"마지막 좌표 오류: {error_code}"

        self.advance_component(
            component,
            TransitionCategory.COMPONENT_FATAL,
            (
                f"{component.name} 좌표 획득 실패 확정: "
                f"error_code={component.error_code}, last_error={error_code}, "
                "다음 Component 관찰 시작"
            ),
            (
                f"마지막 Component {component.name} 좌표 획득 실패 확정: "
                f"error_code={component.error_code}, last_error={error_code}, "
                f"attempt={component.attempt_count}/{self.max_attempts}"
            ),
        )

    def handle_grasp_failure(self):
        """Recover and retry a failed grasp within the configured attempt limit."""
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

            self.fail_task(
                component.error_code,
                component.detail,
                (
                    f"{component.name} 안전 복구 실패: "
                    f"{component.error_code}, {component.detail}"
                ),
                restart_allowed=False,
            )
            return

        if component.attempt_count < self.max_attempts:
            self.transition_to(
                State.OBSERVE,
                TransitionCategory.RETRY,
                (
                    f"{component.name} 파지 실패 후 안전 복구 완료, "
                    f"next_attempt={component.attempt_count + 1}/"
                    f"{self.max_attempts}"
                ),
            )
            return

        component.status = "FAILED"
        component.ended_at = datetime.now(timezone.utc)
        component.error_code = "max_attempts"
        component.detail = "파지 실패로 최대 시도 횟수 도달"

        self.advance_component(
            component,
            TransitionCategory.COMPONENT_FATAL,
            (
                f"{component.name} 파지 실패 확정: "
                f"error_code={component.error_code}, 다음 Component 관찰 시작"
            ),
            (
                f"마지막 Component {component.name} 파지 실패 확정: "
                f"error_code={component.error_code}, "
                f"attempt={component.attempt_count}/{self.max_attempts}"
            ),
        )

    def request_inspection(self):
        if self.pending_future is not None:
            raise RuntimeError("이미 진행 중인 서비스 요청이 있습니다.")

        request = InspectKit.Request()

        # Class and count arrays must retain the same order.
        names = list(self.expected_counts)
        request.expected_classes = names
        request.expected_counts = [self.expected_counts[name] for name in names]
        request.max_age_sec = self.max_age_sec

        self.pending_future = self.inspect_client.call_async(request)
        self.request_deadline = time.monotonic() + self.inspect_timeout

        self.get_logger().info(f"검사 요청: {self.expected_counts}")

    def fail_inspection(self, error_code: str, detail: str):
        # ERROR means inspection was unavailable; FAIL means a valid mismatch.
        self.inspection_result = {
            "result": "ERROR",
            "expected_counts": dict(self.expected_counts),
            "actual_counts": None,
            "missing": [],
            "unexpected": [],
            "detection_age": None,
            "inspected_at": datetime.now(timezone.utc).isoformat(),
        }

        self.fail_task(
            error_code,
            detail,
            f"최종 검사 처리 실패: {error_code}, {detail}",
        )

    def check_inspection_response(self):
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
            names = list(self.expected_counts)

            if not math.isfinite(age) or age < 0 or age > self.max_age_sec:
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

        self.transition_to(
            State.REPORT,
            TransitionCategory.NORMAL,
            f"최종 검사 완료: {self.inspection_result['result']}",
        )

    def publish_task_status(self, task_status="RUNNING"):
        message = TaskStatus()
        message.task_id = self.task_id
        message.state = self.state.name
        message.task_status = task_status
        message.kit_type = self.kit_type
        message.component_total = len(self.components)

        message.current_component = ""
        message.current_component_index = -1

        if self.state in {State.OBSERVE, State.EXECUTE} and self.component_index < len(
            self.components
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
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("시각에는 timezone이 있어야 합니다.")

        message = Time()
        message.sec = int(value.timestamp())
        message.nanosec = value.microsecond * 1000
        return message

    def publish_component_result(self, component):
        if component.index in self.published_component_indices:
            return

        attempts = []

        for attempt in component.attempts:
            attempts.append(
                {
                    "attempt_no": attempt.attempt_no,
                    "status": attempt.status,
                    "started_at": attempt.started_at.isoformat(),
                    "ended_at": attempt.ended_at.isoformat(),
                    "failed_stage": attempt.failed_stage,
                    "error_code": attempt.error_code,
                    "detail": attempt.detail,
                }
            )

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

        self.published_component_indices.add(component.index)

    def finalize_components(self):
        """Preserve finished results and finalize interrupted or unstarted components."""
        now = datetime.now(timezone.utc)

        for component in self.components:
            if component.status not in {"SUCCESS", "FAILED", "SKIPPED"}:
                if component.attempts:
                    component.status = "FAILED"
                    component.error_code = self.error_code or "task_aborted"
                    component.detail = self.detail

                    attempt = component.attempts[-1]

                    if attempt.ended_at is None:
                        attempt.status = "FAILED"
                        attempt.ended_at = now
                        attempt.error_code = component.error_code
                        attempt.detail = component.detail
                        attempt.failed_stage = self.failure_stage
                else:
                    component.status = "SKIPPED"
                    component.started_at = now
                    component.error_code = "task_aborted"
                    component.detail = "작업 중단으로 실행하지 않음"

                component.ended_at = now

            self.publish_component_result(component)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    node.motion = Motion(node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.timer_tick()

    except KeyboardInterrupt:
        pass

    finally:
        # Motion owns a private MoveIt executor and must stop before ROS shutdown.
        try:
            if (
                node.motion is not None
                and hasattr(node.motion, "shutdown")
            ):
                node.motion.shutdown()
        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()