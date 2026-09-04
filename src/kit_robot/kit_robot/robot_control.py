import os
import time
import sys
import json
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [4.00, 38.00, 64.00, -0.1, 78.0, 4]
JHOME_POS = [0, -30, 90, 0, 90, 0]
PLACE_POSITIONS = {
    "pos1": [309.455, -164.533, 314.995, 168.498, 179.790, 168.799],
    "pos2": [677.722, -154.152, 306.509, 39.190, 179.813, 39.228],
    "pos3": [686.782, 145.015, 301.718, 33.625, 179.609, 33.701],
}
PLACE_LIFT = 250.0
PLACE_Z_OFFSET = 50.0
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)




class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.init_robot()

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.ui_pub = self.create_publisher(String, "/ui/current_task", 10)
        self._publish_task(None, None)

    def _publish_task(self, target, pos):
        data = {}
        if target:
            data["target"] = target
        if pos:
            data["pos"] = pos
        try:
            self.ui_pub.publish(String(data=json.dumps(data)))
        except Exception as e:
            self.get_logger().warn(f"_publish_task failed (non-critical): {e}")

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future, timeout_sec=60.0)
        if not rclpy.ok():
            return
        if get_keyword_future.result() is not None and get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()

            message = get_keyword_result.message
            if "/" in message:
                obj_part, dst_part = message.split("/", 1)
                tools = obj_part.split()
                dests = dst_part.split()
            else:
                tools = message.split()
                dests = []

            for i, target in enumerate(tools):
                dest = dests[i] if i < len(dests) else None
                self._publish_task(target, dest)

                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn("No target position")
                    continue
                self.get_logger().info(f"target position: {target_pos} -> place: {dest}")
                self.pick_and_place_target(target_pos, dest)
                self.init_robot()

            self._publish_task(None, None)

        else:
            # get_keyword 는 실패 사유를 message 에 담아 돌려준다(성공 시엔 키워드).
            result = get_keyword_future.result()
            reason = result.message if result is not None else "no_response"
            self.get_logger().warn(f"get_keyword 실패: {reason or 'no keyword detected'}")
            if reason == "openai_quota_exhausted":
                self.get_logger().error(
                    "OpenAI 크레딧 소진 — 충전 필요: "
                    "https://platform.openai.com/settings/organization/billing"
                )
            return

    def get_target_pos(self, target):
        target_pos = None
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)
        if not rclpy.ok():
            return None

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            target_pos = list(td_coord[:3]) + robot_posx[3:]
        return target_pos

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos, dest=None):
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()

        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)
        mwait()

        lift_pos = target_pos[:2] + [target_pos[2] + PLACE_LIFT] + target_pos[3:]
        movel(lift_pos, vel=VELOCITY, acc=ACC)
        mwait()

        if dest in PLACE_POSITIONS:
            place_pos = list(PLACE_POSITIONS[dest])
            place_pos[2] += PLACE_Z_OFFSET
            movel(place_pos, vel=VELOCITY, acc=ACC)
            mwait()
        else:
            self.get_logger().warn(f"Unknown place target '{dest}', releasing at lift position")

        gripper.open_gripper()
        while rclpy.ok() and gripper.get_status()[0]:
            time.sleep(0.5)


def main(args=None):
    node = RobotController()
    try:
        while rclpy.ok():
            node.robot_control()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
