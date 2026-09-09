#!/usr/bin/env python3
"""가상 pick & place 데모 — 팔 모션 + 그리퍼를 한 사이클 수행한다.

bringup.launch.py 가 올린 스택(DRCF 에뮬레이터 + dsr_controller2 + gripper_virtual_node)만
있으면 카메라 / YOLO / 음성 없이 돌아간다. virtual 모드에서 그리퍼가 실제로 닫히고 열리는지
(= RViz 에서 손가락이 움직이는지) 눈으로 확인하는 것이 목적.

    ros2 launch m0609_rg2_bringup bringup.launch.py     # 터미널 1
    ros2 run m0609_rg2_bringup pick_place_demo.py       # 터미널 2

그리퍼는 /onrobot/sendCommand 로만 제어한다 — virtual 이면 gripper_virtual_node 가,
real 이면 OnRobot 드라이버가 같은 서비스를 제공하므로 이 스크립트는 모드를 모른다.
"""
import sys

import rclpy
import DR_init
from onrobot_rg_msgs.srv import SetCommand

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
VELOCITY, ACC = 40, 40

# 관절 공간 웨이포인트 [j1..j6] (deg). 태스크 공간 IK 를 타지 않아 특이점 / 도달불가 없이
# 어떤 셀 배치에서도 돌아간다. 실제 집기 좌표는 비전 파이프라인이 정할 몫이라 여기선 고정값.
J_HOME = [0, 0, 90, 0, 90, 0]
J_PICK_ABOVE = [-30, 15, 85, 0, 80, 0]
J_PLACE_ABOVE = [30, 15, 85, 0, 80, 0]

APPROACH_MM = 100.0  # 웨이포인트에서 수직으로 내려가는 거리 (mm)
GRIP_WIDTH_DMM = 400  # 물체를 문 상태의 그리퍼 너비 (1/10 mm) — 완전히 닫지 않는다

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node('pick_place_demo', namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait
except ImportError as e:
    print(f'Error importing DSR_ROBOT2: {e}', file=sys.stderr)
    sys.exit(1)


class Gripper:
    """/onrobot/sendCommand 클라이언트. 서비스가 애니메이션 완료까지 블로킹하므로 별도 대기 불필요."""

    def __init__(self, node):
        self._node = node
        self._client = node.create_client(SetCommand, '/onrobot/sendCommand')
        while not self._client.wait_for_service(timeout_sec=3.0):
            node.get_logger().info('waiting for /onrobot/sendCommand ...')

    def command(self, value):
        """value: 'o' | 'c' | 목표 너비(1/10 mm) 정수."""
        req = SetCommand.Request()
        req.command = str(value)
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f'gripper command {value!r} failed: {result}')
        self._node.get_logger().info(f'gripper -> {value}')


def descend_and_return(depth_mm):
    """현재 자세에서 수직으로 depth_mm 내려갔다가, 호출자가 원위치할 수 있도록 원래 pose 를 돌려준다."""
    start = list(get_current_posx()[0])
    lowered = list(start)
    lowered[2] -= depth_mm
    movel(lowered, vel=VELOCITY, acc=ACC)
    mwait()
    return start


def main():
    node = rclpy.create_node('pick_place_demo_client')
    gripper = Gripper(node)
    log = node.get_logger()

    try:
        log.info('[1/6] home')
        movej(J_HOME, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.command('o')

        log.info('[2/6] move above pick')
        movej(J_PICK_ABOVE, vel=VELOCITY, acc=ACC)
        mwait()

        log.info('[3/6] descend & grip')
        above_pick = descend_and_return(APPROACH_MM)
        gripper.command(GRIP_WIDTH_DMM)
        movel(above_pick, vel=VELOCITY, acc=ACC)
        mwait()

        log.info('[4/6] move above place')
        movej(J_PLACE_ABOVE, vel=VELOCITY, acc=ACC)
        mwait()

        log.info('[5/6] descend & release')
        above_place = descend_and_return(APPROACH_MM)
        gripper.command('o')
        movel(above_place, vel=VELOCITY, acc=ACC)
        mwait()

        log.info('[6/6] home')
        movej(J_HOME, vel=VELOCITY, acc=ACC)
        mwait()
        log.info('pick & place cycle done')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
