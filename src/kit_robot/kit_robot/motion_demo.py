"""
Motion 호출 계약 확인용 데모.
ROS·DSR 연결이나 실제 이동은 하지 않는다.

Controller(motion=MotionDemo())로 주입한다.
모든 동작은 즉시 완료된 것으로 처리하며 pick은 항상 성공한다.
자세는 호출 확인용 가상 값이므로
실제 카메라 검출과 결합한 좌표 변환 검증에는 사용하지 않는다.
"""


class MotionDemo:
    """Controller가 사용하는 Motion 메서드의 이름과 반환 형식을 제공한다."""

    def __init__(self):
        # 가상 TCP 자세: [x, y, z, rx, ry, rz], mm·deg·ZYZ.
        self._current_pose = [0.0] * 6

    def move_home(self) -> None:
        """대기 자세 이동과 그리퍼 개방이 완료된 것으로 처리한다."""
        self._current_pose = [0.0] * 6
        print("[MotionDemo] move_home: 대기 자세, 그리퍼 개방")

    def move_to_observation_pose(self) -> None:
        """관찰 자세 이동이 완료된 것으로 처리한다."""
        self._current_pose = [0.0] * 6
        print("[MotionDemo] move_to_observation_pose: 관찰 자세 도착")

    def move_to_inspection_pose(self) -> None:
        """검사 자세 이동이 완료된 것으로 처리한다."""
        self._current_pose = [0.0] * 6
        print("[MotionDemo] move_to_inspection_pose: 검사 자세 도착")

    def get_current_pose(self) -> list[float]:
        """가상 TCP 자세 6개를 복사해 반환한다."""
        print(f"[MotionDemo] get_current_pose: {self._current_pose}")
        return self._current_pose.copy()

    def pick_component(
        self,
        component_name: str,
        target_pose: list[float],
    ) -> bool:
        """파지 호출을 출력하고 항상 성공을 반환한다."""
        print(
            f"[MotionDemo] pick_component: {component_name}, "
            f"target_pose={target_pose}"
        )
        return True

    def place_component(
        self,
        component_name: str,
        slot_name: str,
    ) -> None:
        """슬롯 이름을 출력한다. 실제 슬롯 좌표는 조회하지 않는다."""
        print(
            f"[MotionDemo] place_component: {component_name}, "
            f"slot={slot_name}"
        )

    def recover_to_safe_pose(self) -> None:
        """그리퍼 개방과 안전 복귀가 완료된 것으로 처리한다."""
        self._current_pose = [0.0] * 6
        print("[MotionDemo] recover_to_safe_pose: 개방 및 안전 복귀")


def main():
    """로봇 없이 일곱 메서드의 호출 예시를 실행한다."""
    motion = MotionDemo()
    motion.move_home()
    motion.move_to_observation_pose()
    pose = motion.get_current_pose()
    if motion.pick_component("컵라면", pose):
        motion.place_component("컵라면", "slot_1")
    motion.move_to_inspection_pose()
    motion.recover_to_safe_pose()


if __name__ == "__main__":
    main()
