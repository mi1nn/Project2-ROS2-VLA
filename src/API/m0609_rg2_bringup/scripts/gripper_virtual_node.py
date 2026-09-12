#!/usr/bin/env python3
"""가상 OnRobot RG2 — DRCF 에뮬레이터(virtual) 모드에서 실 드라이버를 대신한다.

실 드라이버(onrobot_rg_control/OnRobotRGControllerServer)와 같은 인터페이스를 구현해,
호출자(pick&place 노드 등)가 모드에 무관하게 동일한 코드로 그리퍼를 쓴다.

  service : /onrobot/sendCommand (onrobot_rg_msgs/SetCommand)
            'o' = 열기, 'c' = 닫기, 정수 = 목표 너비(1/10 mm, 0..1100)
            요청은 애니메이션이 목표에 도달할 때까지 블로킹 — 실 드라이버의 busy 대기와 동일 의미.
  topic   : /onrobot_joint_states (sensor_msgs/JointState, 50 Hz)
            실 드라이버와 같은 조인트명(finger_joint). gripper_joint_state_publisher 가
            URDF 조인트명(rg2_ prefix)으로 바꿔 /gripper_joint_states 로 재발행한다.
            나머지 그리퍼 조인트는 URDF 에서 finger_joint 를 mimic 하므로 발행하지 않는다.
"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from onrobot_rg_msgs.srv import SetCommand

# RG2 4절 링크 파라미터 — 실 드라이버(OnRobotRGControllerServer)와 동일 값.
# 너비(m) ↔ finger_joint 각(rad) 변환에 쓴다. 그대로 두어야 virtual/real 이 같은 각도를 낸다.
L1, L3 = 0.108505, 0.055
THETA1, THETA3 = 1.41371, 0.76794
DY = -0.0144
MAX_WIDTH_DMM = 1100  # RG2 최대 개방 폭 (1/10 mm)

# URDF(onrobot_rg2_model_macro.xacro)의 rg2_finger_joint limit. 링크 기구학상 최대 개방폭은
# URDF limit 보다 약간 좁아 열림 각은 여기에 닿지 않지만, 닫힘 각은 넘어서므로 클램프가 필요하다.
JOINT_MIN, JOINT_MAX = -0.558505, 0.785398

PUBLISH_RATE = 1.0 / 50.0  # 50 Hz — 실 드라이버와 동일
SPEED = 1.0                # rad/s 애니메이션 속도
DONE_TOL = 0.01            # rad — 목표 도달 판정


def width_to_joint(width_m):
    """그리퍼 너비(m) → finger_joint 각(rad). URDF limit 으로 클램프."""
    arg = ((width_m / 2) - DY - L1 * math.cos(THETA1)) / L3
    angle = math.acos(max(-1.0, min(1.0, arg))) - THETA3
    return max(JOINT_MIN, min(JOINT_MAX, angle))


class GripperVirtualNode(Node):

    def __init__(self):
        super().__init__('gripper_virtual_node')
        cb_group = ReentrantCallbackGroup()

        self._pub = self.create_publisher(JointState, '/onrobot_joint_states', 10)
        self._position = width_to_joint(MAX_WIDTH_DMM / 10000.0)  # 열린 상태로 기동
        self._target = self._position
        self._lock = threading.Lock()

        self.create_timer(PUBLISH_RATE, self._publish_cb, callback_group=cb_group)
        self.create_service(
            SetCommand, '/onrobot/sendCommand', self._send_command_cb, callback_group=cb_group,
        )
        self.get_logger().info('GripperVirtualNode ready — /onrobot/sendCommand')

    def _send_command_cb(self, req, res):
        command = str(req.command)
        if command == 'c':
            width_dmm = 0
        elif command == 'o':
            width_dmm = MAX_WIDTH_DMM
        else:
            # 실 드라이버와 동일: 정수는 목표 너비(1/10 mm). 그 외는 거부.
            try:
                width_dmm = min(MAX_WIDTH_DMM, max(0, int(command)))
            except ValueError:
                res.success = False
                res.message = f'Unknown command: {command!r}'
                return res

        target = width_to_joint(width_dmm / 10000.0)
        with self._lock:
            self._target = target

        # 애니메이션이 목표에 도달할 때까지 블로킹. 호출자가 곧바로 다음 모션으로 넘어가
        # RViz 상 그리퍼가 물체를 놓친 것처럼 보이는 것을 막는다.
        while rclpy.ok():
            with self._lock:
                if abs(self._position - target) < DONE_TOL:
                    break
            time.sleep(PUBLISH_RATE)

        res.success = True
        res.message = ''
        return res

    def _publish_cb(self):
        with self._lock:
            diff = self._target - self._position
            step = SPEED * PUBLISH_RATE
            if abs(diff) <= step:
                self._position = self._target
            else:
                self._position += step * (1.0 if diff > 0 else -1.0)
            pos = self._position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_joint']
        msg.position = [pos]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperVirtualNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # launch 종료(SIGINT) 시 rclpy 기본 시그널 핸들러가 이미 context 를 shutdown 한 뒤
        # ExternalShutdownException 이 올라온다 → 중복 호출 시 RCLError('rcl_shutdown already
        # called'). ok() 가드로 한 번만 호출.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
