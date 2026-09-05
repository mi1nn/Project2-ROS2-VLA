import os
import yaml

from ament_index_python.packages import get_package_share_directory
from .onrobot import RG


class Motion:
    def __init__(self, node):

        if node is None:
            raise ValueError("ROS 2 node is required for DSR initialization.")

        config_path = os.path.join(
            get_package_share_directory("kit_robot"), "config", "motion.yaml"
        )

        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)["motion"]

        self.positions = config["positions"]

        try:
            from DSR_ROBOT2 import (
                movej,
                movel,
                wait,
                get_current_posx,
                posx,
                posj,
                # trans,
                # set_tool,
                # set_tcp,
                DR_BASE,
                DR_TOOL,
            )
        except ImportError as e:
            raise ImportError("failed import dsr library") from e

        # if set_tool("Tool Weight") != 0:
        #     raise RuntimeError(
        #         "Failed to set tool: Tool Weight"
        #     )

        # if set_tcp("GripperDA_v1") != 0:
        #     raise RuntimeError(
        #         "Failed to set TCP: GripperDA_v1"
        #     )

        self.movej = movej
        self.movel = movel
        self.wait = wait
        self.get_current_posx = get_current_posx
        self.posx = posx
        self.posj = posj
        # self.trans = trans
        self.DR_BASE = DR_BASE
        self.DR_TOOL = DR_TOOL

        self.rg = RG("rg2", "192.168.1.1", 502)

    def home(self):
        config = self.positions["home"]
        if config["type"] == "joint":
            home_pos = self.posj(config["pos"])
        else:
            raise TypeError("home position must be 'joint'")

        return self.movej(home_pos, vel=config["joint_vel"], acc=config["joint_acc"])

    def pick_camera(self):
        config = self.positions["pick_camera"]
        if config["type"] == "joint":
            pick_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("pick_camera position must be 'joint'")

        return self.movej(
            pick_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"]
        )

    def place_camera(self):
        config = self.positions["place_camera"]
        if config["type"] == "joint":
            place_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("place_camera position must be 'joint'")

        return self.movej(
            place_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"]
        )

    def current_posx(self):
        (pose,) = self.get_current_posx(ref=self.DR_BASE)
        if pose is None:
            raise RuntimeError("Failed to get current posx")
        return list(pose)

    def move_linear(self, target_pose, vel=100, acc=200):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")
        dsr_pose = self.posx([float(value) for value in pose])

        result = self.movel(dsr_pose, vel=vel, acc=acc, ref=self.DR_BASE)
        if result != 0:
            raise RuntimeError(f"movel failed: result={result}, pose={pose}")
        return result

    def pick(self, target_pose, vel=100, acc=200, approach_height=100):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")
        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        self.rg.open_gripper()
        self.wait(2.0)
        self.move_linear(pick_pose_up, vel=vel, acc=acc)
        self.wait(0.5)
        self.move_linear(pick_pose_down, vel=vel, acc=acc)
        self.rg.close_gripper()
        self.wait(2.0)

        gripper_width = self.rg.get_width()

        self.move_linear(pick_pose_up, vel=vel, acc=acc)

        print(f'gripper_width: {gripper_width}')

        return gripper_width > 13
