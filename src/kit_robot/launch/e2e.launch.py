"""docs/07 Controller E2E 를 한 번에 띄운다.

기동 순서: MoveIt2/M0609 -> RealSense -> get_command -> controller -> position_estimation

순서는 TimerAction 누적 지연으로 만든다. ROS 2 launch 에는 "이 노드가 쓸 만해지면
다음 노드" 를 표현하는 표준 수단이 없고(OnProcessStart 는 프로세스가 뜬 순간이라
기다리는 의미가 없다), 실제로 기다려야 하는 대상은 프로세스가 아니라 MoveIt action
server 와 RealSense 의 첫 포인트클라우드다. 그래서 지연을 launch 인자로 빼 두었다 —
현장 PC 가 느리거나 로봇 부팅이 오래 걸리면 값만 올린다.

    ros2 launch kit_robot e2e.launch.py
    ros2 launch kit_robot e2e.launch.py moveit_ready_sec:=25.0 host:=192.168.1.100

DB(postgres/mongodb)·비전 컨테이너는 docker compose 소관이라 여기 없다.
먼저 띄운다: docker compose up -d postgres mongodb vision
환경변수(ROS_DOMAIN_ID, RMW_IMPLEMENTATION)는 launch 를 실행하는 셸에서 준다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _after(*configs):
    """launch 시작 기준 누적 지연(초)."""
    terms = []
    for config in configs:
        if terms:
            terms.append(" + ")
        terms.append(config)
    return PythonExpression(terms)


def generate_launch_description():
    moveit_ready_sec = LaunchConfiguration("moveit_ready_sec")
    camera_ready_sec = LaunchConfiguration("camera_ready_sec")
    node_gap_sec = LaunchConfiguration("node_gap_sec")

    controller_params = os.path.join(
        get_package_share_directory("kit_robot"), "resource", "controller.yaml"
    )

    # ~/.bashrc 의 robotmoveit alias 와 같은 인자다.
    robot_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dsr_bringup2"),
                "launch",
                "dsr_bringup2_moveit.launch.py",
            )
        ),
        launch_arguments={
            "name": LaunchConfiguration("name"),
            "model": LaunchConfiguration("model"),
            "mode": LaunchConfiguration("mode"),
            "host": LaunchConfiguration("host"),
            "gripper": LaunchConfiguration("gripper"),
        }.items(),
    )

    # ~/.bashrc 의 realsense alias 와 같은 파라미터다. 노드 이름 camera, 네임스페이스
    # 루트 — motion.yaml 의 octomap.cloud_in(/camera/depth/color/points)과 맞춘다.
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="/",
        output="screen",
        parameters=[
            {
                "enable_color": True,
                "enable_depth": True,
                "depth_module.depth_profile": "848x480x15",
                "rgb_camera.color_profile": "1280x720x15",
                "align_depth.enable": True,
                "enable_rgbd": True,
                "enable_sync": True,
                "pointcloud.enable": True,
                "pointcloud.stream_filter": 2,
                "enable_accel": False,
                "enable_gyro": False,
                "initial_reset": True,
            }
        ],
    )

    get_command = Node(
        package="kit_voice",
        executable="get_command",
        output="screen",
    )

    # 노드 이름·네임스페이스는 controller.py 가 직접 "/dsr01/controller" 로 잡는다.
    # 여기서 name/namespace 를 주면 controller.yaml 의 키와 어긋난다.
    controller = Node(
        package="kit_robot",
        executable="controller",
        output="screen",
        parameters=[controller_params],
    )

    position_estimation = Node(
        package="kit_robot",
        executable="position_estimation",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="dsr01"),
            DeclareLaunchArgument("model", default_value="m0609"),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("host", default_value="192.168.1.100"),
            DeclareLaunchArgument("gripper", default_value="rg2"),
            DeclareLaunchArgument(
                "moveit_ready_sec",
                default_value="3.0",
                description=(
                    "로봇 bringup + move_action/execute_trajectory 가 올라올 때까지. "
                    "Motion 의 server_ready_timeout_sec(15초)이 여기서 시작하므로 "
                    "짧으면 controller 가 초기화에 실패한다."
                ),
            ),
            DeclareLaunchArgument(
                "camera_ready_sec",
                default_value="3.0",
                description=(
                    "RealSense initial_reset 후 첫 포인트클라우드까지. "
                    "카메라 USB 재연결이 느리면 올린다."
                ),
            ),
            DeclareLaunchArgument(
                "node_gap_sec",
                default_value="2.0",
                description="음성·controller·좌표 노드 사이 간격",
            ),
            # 1. 로봇 + MoveIt2
            robot_moveit,
            # 2. 카메라
            TimerAction(period=moveit_ready_sec, actions=[realsense]),
            # 3. 음성
            TimerAction(
                period=_after(moveit_ready_sec, camera_ready_sec),
                actions=[get_command],
            ),
            # 4. controller. Motion 초기화가 여기서 MoveIt·TF·RG2 를 잡는다.
            TimerAction(
                period=_after(moveit_ready_sec, camera_ready_sec, node_gap_sec),
                actions=[controller],
            ),
            # 5. 좌표. controller 가 /get_component_pose 를 실제로 부르는 건 음성
            # 명령을 받은 뒤 OBSERVE 단계라, 늦게 떠도 service_ready_timeout_sec
            # (20초) 안에만 들어오면 된다.
            TimerAction(
                period=_after(
                    moveit_ready_sec, camera_ready_sec, node_gap_sec, node_gap_sec
                ),
                actions=[position_estimation],
            ),
        ]
    )
