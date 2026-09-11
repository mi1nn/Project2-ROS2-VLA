"""웨이크워드를 말하기 전에는 IDLE 을 벗어나지 않아야 한다.

Controller 전체는 rclpy·MoveIt·RG2 없이 생성할 수 없으므로, IDLE 게이트를 이루는
세 메서드만 최소 스텁에 바인딩해서 검사한다. ROS 환경에서 실행한다.

    PYTHONPATH=src/kit_robot python3 -m pytest src/kit_robot/test/test_wakeword_gate.py -q
"""

from kit_robot.controller import Controller, State


class _Gate:
    """handle_idle / check_wakeword / on_wakeword 가 건드리는 것만 갖춘 스텁."""

    on_wakeword = Controller.on_wakeword
    check_wakeword = Controller.check_wakeword
    handle_idle = Controller.handle_idle

    def __init__(self):
        self.state = State.IDLE
        self.motion = None
        self.wakeword_pending = False
        self.task_id = ""
        self.transitions = []

    def get_logger(self):
        return self

    def info(self, *_args, **_kwargs):
        pass

    def transition_to(self, next_state, _category, _reason):
        self.transitions.append(next_state)
        self.state = next_state


def test_idle_stays_until_wakeword():
    gate = _Gate()

    gate.handle_idle(True)
    assert gate.transitions == [], "IDLE 진입만으로 넘어가면 안 된다"

    for _ in range(50):
        gate.handle_idle(False)
    assert gate.transitions == [], "호출어 없이 tick 만으로 넘어가면 안 된다"

    gate.on_wakeword(None)
    gate.handle_idle(False)
    assert gate.transitions == [State.LISTEN], gate.transitions
    # task_id 는 IDLE 진입이 아니라 웨이크워드 시점에 생긴다.
    assert gate.task_id.startswith("TASK-"), gate.task_id


def test_wakeword_during_task_is_ignored():
    gate = _Gate()
    gate.handle_idle(True)

    gate.state = State.EXECUTE
    gate.on_wakeword(None)
    assert gate.wakeword_pending is False, "작업 중 발화를 쌓아두면 안 된다"

    gate.state = State.IDLE
    gate.handle_idle(True)
    gate.handle_idle(False)
    assert gate.transitions == [], "작업 중 발화가 다음 작업을 자동 시작하면 안 된다"
