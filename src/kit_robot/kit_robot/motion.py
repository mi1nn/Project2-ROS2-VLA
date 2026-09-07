import os
import yaml
import json

from ament_index_python.packages import get_package_share_directory
from .onrobot import RG


import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

class Motion:
    def __init__(self, node):

        if node is None:
            raise ValueError("ROS 2 node is required for DSR initialization.")

        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node

        config_path = os.path.join(
            get_package_share_directory("kit_robot"), "config", "motion.yaml"
        )

        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)["motion"]

        self.positions = config["positions"]
        self.place_config = config['place']
        self.place_slots = self.place_config['slots']

        grasp_params_path = os.path.join(
            get_package_share_directory('kit_robot'),
            'resource',
            'grasp_params.json',
            )

        with open(grasp_params_path, 'r', encoding='utf-8') as file:
            self.grasp_params = json.load(file)

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

    def move_home(self):
        config = self.positions["home"]
        if config["type"] == "joint":
            home_pos = self.posj(config["pos"])
        else:
            raise TypeError("home position must be 'joint'")

        result = self.movej(home_pos, vel=config["joint_vel"], acc=config["joint_acc"])
        if result != 0:
            raise RuntimeError(f"move_home failed: result={result}")

        return result

    def move_to_observation_pose(self):
        config = self.positions["observation_pose"]
        if config["type"] == "joint":
            pick_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("observation_pose must be 'joint'")

        result = self.movej(pick_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"])
        if result != 0:
            raise RuntimeError(f'move_to_observation_pose failed: result={result}')

        return result

    def move_to_inspection_pose(self):
        config = self.positions["inspection_pose"]
        if config["type"] == "joint":
            place_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("inspection_pose must be 'joint'")

        result = self.movej(place_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"])
        if result != 0:
            raise RuntimeError(f'move_to_inspection_pose failed: result={result}')

        return result

    def get_current_pose(self):
        pose = self.get_current_posx(ref=self.DR_BASE)[0]
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

    def pick_component(self, component_name, target_pose, vel=100, acc=200):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")

        params = self.grasp_params.get(component_name, self.grasp_params['_default'])
        open_width = params['width']
        grip_force = params['force']
        approach_height = params['approach']

        result = False
        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        print(
            f"Pick component: {component_name}, "
            f"width={open_width}, "
            f"force={grip_force}, "
            f"approach={approach_height}"
        )

        for i in range(5):
            self.rg.move_gripper(open_width, force_val=grip_force)
            self.wait(2.0)
            self.move_linear(pick_pose_up, vel=vel, acc=acc)
            self.wait(0.5)
            self.move_linear(pick_pose_down, vel=vel, acc=acc)
            self.rg.close_gripper(force_val=grip_force)
            self.wait(2.0)

            gripper_width = self.rg.get_width()

            self.move_linear(pick_pose_up, vel=vel, acc=acc)
            print(f'gripper_width: {gripper_width}')

            if gripper_width > 13:
                print('Success to grip object')
                result = True
                break
            else:
                print(f'{i+1} try, Failed to grip object')
                

            if i == 4:
                print('Failed to grip object in all try')
                result = False

        return result

    def place_component(self, component_name, slot_name, approach_height=100):
        if slot_name not in self.place_slots:
            raise ValueError(f'Unknown place slot: {slot_name}')
        place_pose_down = list(self.place_slots[slot_name]['pos'])

        if len(place_pose_down) != 6:
            raise ValueError(f"{slot_name} pose must be [x, y, z, rx, ry, rz]")

        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height

        vel = self.place_config['linear_vel']
        acc = self.place_config['linear_acc']

        print(
            f"Place component: {component_name}, "
            f"slot={slot_name}, "
            f"pose={place_pose_down}"
        )

        self.move_linear(place_pose_up, vel=vel, acc=acc)
        self.wait(0.5)
        self.move_linear(place_pose_down, vel=vel, acc=acc)
        self.rg.open_gripper()
        self.wait(2.0)
        self.move_linear(place_pose_up,vel=vel,acc=acc)

        print(f"Complete place: {component_name} -> {slot_name}")

    def recover_to_safe_pose(self):
        print("Recovering to safe pose")
        self.rg.open_gripper()
        self.wait(2.0)
        result = self.move_home()

        if result != 0:
            raise RuntimeError(f"Failed to recover home: result={result}")

        print("Complete recovery to safe pose")
