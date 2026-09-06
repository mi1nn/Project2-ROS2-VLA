import os
import yaml
import DR_init

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

        # DSR_ROBOT2는 import 시점에 DR_init.__dsr__node로 서비스 client를 만들기
        # 때문에 import 전에 반드시 설정해야 한다.
        _set_dr_init(node)

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

        return self.movej(home_pos, vel=config["joint_vel"], acc=config["joint_acc"])

    def move_to_observation_pose(self):
        config = self.positions["observation_pose"]
        if config["type"] == "joint":
            pick_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("observation_pose must be 'joint'")

        return self.movej(
            pick_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"]
        )

    def move_to_inspection_pose(self):
        config = self.positions["inspection_pose"]
        if config["type"] == "joint":
            place_camera_pos = self.posj(config["pos"])
        else:
            raise TypeError("inspection_pose must be 'joint'")

        return self.movej(
            place_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"]
        )

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

    def pick_component(self, target_pose, vel=100, acc=200, approach_height=100):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")

        result = 0
        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        for i in range(5):
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

            if gripper_width > 13:
                print('Success to grip object')
                result = 1
                break
            else:
                print(f'{i+1} try, Failed to grip object')
                

            if i == 4:
                print('Failed to grip object in all try')
                result = -1

        return result

    '''
    미구현 부분
    중간 점검을 위해 log만 송출
    '''
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
