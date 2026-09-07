import os
import yaml
import rclpy
import DR_init
import json

from ament_index_python.packages import get_package_share_directory
from .onrobot import RG

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


def _set_dr_init(node):
    '''DR_init.__dsr__* 설정. class 본문에서 직접 쓰면 name mangling으로
    엉뚱한 속성(_Motion__dsr__id 등)에 저장되므로 반드시 모듈 레벨 함수로 둔다.'''
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node


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

        # DSR_ROBOT2는 서비스 이름을 자기 노드 네임스페이스 기준 상대경로로 연다
        # (예: "dsr_controller2/motion/move_joint").  robot_id는 서비스 이름에
        # 안 들어가므로, Controller 노드(네임스페이스 없음)를 그대로 넘기면
        # /dsr01/dsr_controller2/... 를 못 찾고 영원히 대기한다.
        # 레퍼런스(robot_control.py)처럼 namespace=ROBOT_ID인 전용 노드를 따로 둔다.
        # DSR_ROBOT2는 import 시점에 DR_init.__dsr__node로 서비스 client를 만들기
        # 때문에 import 전에 반드시 설정해야 한다.
        self._dsr_node = rclpy.create_node("dsr_interface", namespace=ROBOT_ID)
        _set_dr_init(self._dsr_node)

        try:
            from DSR_ROBOT2 import (
                movej,
                movel,
                movesx,
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
        self.movesx = movesx
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
        pose, _ = self.get_current_posx(ref=self.DR_BASE)
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

    def move_arc(self, target_pose, height=100, steps=6, vel=100, acc=200):
        start = self.get_current_pose()
        end = list(target_pose)

        if len(start) != 6 or len(end) != 6:
            raise ValueError(
                "start_pose and end_pose must be [x, y, z, rx, ry, rz]"
            )

        if steps < 2:
            raise ValueError("steps must be at least 2")

        if height < 0:
            raise ValueError("height must be greater than or equal to 0")

        start = [float(value) for value in start]
        end = [float(value) for value in end]

        points = []

        for i in range(1, steps +1):
            t = float(i)/steps

            x = start[0] + (end[0]-start[0])*t
            y = start[1] + (end[1]-start[1])*t 
            z_linear = start[2] + (end[2]-start[2])*t
            z_arc = 4.0*height*t*(1.0-t)
            z = z_linear + z_arc

            rx = start[3] + (end[3]-start[3])*t 
            ry = start[4] + (end[4]-start[4])*t 
            rz = start[5] + (end[5]-start[5])*t 

            points.append(self.posx([x,y,z,rx,ry,rz]))

        result = self.movesx(points, vel=vel, acc=acc, ref=self.DR_BASE)

        if result != 0:
            raise RuntimeError(
            f"move_arc failed: result={result}, "
            f"start={start}, end={end}"
        )

        return result


    def pick_component(self, component_name, target_pose, vel=80, acc=160):
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
            self.wait(5.0)

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

        place_approach = list(self.place_slots['slot_0']['pos'])

        if len(place_pose_down) != 6:
            raise ValueError(f"{slot_name} pose must be [x, y, z, rx, ry, rz]")

        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height
        

        vel = self.place_config['linear_vel']
        acc = self.place_config['linear_acc']

        print('move to approach pose')

        self.move_arc(place_approach, height=100, steps=6, vel=vel, acc=acc)

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
        self.move_linear(place_pose_up, vel=vel, acc=acc)
        self.move_linear(place_approach,vel=vel,acc=acc)

        print(f"Complete place: {component_name} -> {slot_name}")

    def recover_to_safe_pose(self):
        print("Recovering to safe pose")
        self.rg.open_gripper()
        self.wait(2.0)
        result = self.move_home()

        if result != 0:
            raise RuntimeError(f"Failed to recover home: result={result}")

        print("Complete recovery to safe pose")
