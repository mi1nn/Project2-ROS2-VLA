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
                DR_BASE,
                DR_TOOL,
            )
        except ImportError as e:
            raise ImportError("failed import dsr library") from e

        self.movej = movej
        self.movel = movel
        self.wait = wait
        self.get_current_posx = get_current_posx
        self.posx = posx
        self.posj = posj
        # self.trans = trans
        self.DR_BASE = DR_BASE
        self.DR_TOOL = DR_TOOL

        self.gripper = RG("rg2", "192.168.1.1", 502)

    def home(self):
        config = self.positions["home"]
        if config["type"] == "joint":
            home_pos = self.posj(config["pos"])
        else:
            raise TypeError("home position must be 'joint'")

        return self.movej(home_pos, vel=config["joint_vel"], acc=config["joint_acc"])

    def pick_camera(self):
        config = self.positions['pick_camera']
        if config['type'] == 'joint':
            pick_camera_pos = self.posj(config['pos'])
        else:
            raise TypeError("pick_camera position must be 'joint'")
        
        return self.movej(pick_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"])
    
    def place_camera(self):
        config = self.positions['place_camera']
        if config['type'] == 'joint':
            place_camera_pos = self.posj(config['pos'])
        else:
            raise TypeError("place_camera position must be 'joint'")
        
        return self.movej(place_camera_pos, vel=config["joint_vel"], acc=config["joint_acc"])