import os
import re
import tempfile

import xacro

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

from dsr_bringup2.controller_config import (
    adjust_dsr_controller_yaml,
    parse_joints_from_urdf,
)

from dsr_bringup2.utils import read_update_rate


# ================================================================
# Helper
# ================================================================
def is_rg2(context):
    return (
        LaunchConfiguration("gripper")
        .perform(context)
        .lower()
        == "rg2"
    )


def robot_namespace(context):
    return (
        LaunchConfiguration("name")
        .perform(context)
        .strip("/")
    )


# ================================================================
# Robot Description
#
# 일반:
#   기존 Doosan URDF 사용
#
# gripper:=rg2:
#   m0609_rg2_bringup의
#   M0609 + RG2 + bracket + D435 통합 URDF 사용
# ================================================================
def generate_robot_description_action(
    context,
    *args,
    **kwargs,
):

    dynamic_yaml = (
        LaunchConfiguration("dynamic_yaml")
        .perform(context)
        .lower()
        == "true"
    )

    model = LaunchConfiguration("model").perform(context)
    color = LaunchConfiguration("color").perform(context)
    name = LaunchConfiguration("name").perform(context)
    host = LaunchConfiguration("host").perform(context)
    rt_host = LaunchConfiguration("rt_host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    update_rate = read_update_rate()

    # ------------------------------------------------------------
    # RG2
    # ------------------------------------------------------------
    if is_rg2(context):

        if model != "m0609":
            raise RuntimeError(
                "gripper:=rg2 구성은 현재 model:=m0609만 지원합니다."
            )

        if not robot_namespace(context):
            raise RuntimeError(
                "gripper:=rg2 사용 시 "
                "name:=dsr01 을 지정해야 합니다."
            )

        xacro_file = os.path.join(
            get_package_share_directory(
                "m0609_rg2_bringup"
            ),
            "urdf",
            "m0609_with_rg2_camera.urdf.xacro",
        )

        print(
            "[INFO] RG2 integrated URDF:",
            xacro_file,
        )

        # 기존 roboton에서 쓰던 통합 xacro를 그대로 사용
        urdf_xml = xacro.process_file(
            xacro_file,
            mappings={
                "host": host,
                "port": str(port),
                "rt_host": rt_host,
                "mode": mode,
                "model": "m0609",
                "update_rate": str(update_rate),
            },
        ).toxml()

        # RG2는 Doosan ros2_control이 아니라
        # 별도 OnRobot Modbus 드라이버로 제어한다.
        #
        # 따라서 dsr_controller2에는 M0609 팔 controller만 사용.
        adjusted_yaml = os.path.join(
            get_package_share_directory(
                "dsr_controller2"
            ),
            "config",
            "dsr_controller2.yaml",
        )

        print(
            "[INFO] RG2 mode - controller YAML:",
            adjusted_yaml,
        )

    # ------------------------------------------------------------
    # 기존 Doosan 구성
    # ------------------------------------------------------------
    else:

        (
            urdf_xml,
            active_joints,
            passive_joints,
        ) = parse_joints_from_urdf(
            model,
            color,
            name,
            host,
            rt_host,
            port,
            mode,
            update_rate,
        )

        print(
            f"[DEBUG] model={model}, "
            f"color={color}, "
            f"name={name}, "
            f"host={host}, "
            f"rt_host={rt_host}, "
            f"port={port}, "
            f"mode={mode}, "
            f"update_rate={update_rate}"
        )

        print(
            f"[DEBUG] active_joints="
            f"{active_joints}"
        )

        print(
            f"[DEBUG] passive_joints="
            f"{passive_joints}"
        )

        if dynamic_yaml:

            original_yaml = os.path.join(
                get_package_share_directory(
                    "dsr_controller2"
                ),
                "config",
                "dsr_controller2.yaml",
            )

            adjusted_yaml = (
                adjust_dsr_controller_yaml(
                    original_yaml,
                    active_joints,
                    passive_joints,
                )
            )

            print(
                "[INFO] Using dynamic YAML:",
                adjusted_yaml,
            )

        else:

            static_yaml = os.path.join(
                get_package_share_directory(
                    "dsr_controller2"
                ),
                "config",
                f"dsr_controller2_{model}.yaml",
            )

            if os.path.exists(static_yaml):

                adjusted_yaml = static_yaml

                print(
                    "[INFO] Using static YAML:",
                    adjusted_yaml,
                )

            else:

                adjusted_yaml = os.path.join(
                    get_package_share_directory(
                        "dsr_controller2"
                    ),
                    "config",
                    "dsr_controller2.yaml",
                )

                print(
                    "[WARN] Model YAML not found. "
                    "Using default:",
                    adjusted_yaml,
                )

    return [
        SetLaunchConfiguration(
            "robot_description",
            urdf_xml,
        ),

        SetLaunchConfiguration(
            "controller_yaml",
            adjusted_yaml,
        ),
    ]


# ================================================================
# DSR emulator / real connection
#
# RG2는 Doosan의 gripper가 아니므로
# DSR 쪽에는 gripper=none 전달
# ================================================================
def run_emulator_fn(context):

    gripper_value = (
        LaunchConfiguration("gripper")
        .perform(context)
        .lower()
    )

    dsr_gripper = (
        "none"
        if gripper_value == "rg2"
        else gripper_value
    )

    return [
        Node(
            package="dsr_bringup2",
            executable="run_emulator",

            namespace=LaunchConfiguration(
                "name"
            ),

            parameters=[{
                "name":
                    LaunchConfiguration("name"),

                "rate":
                    100,

                "standby":
                    5000,

                "command":
                    True,

                "host":
                    LaunchConfiguration("host"),

                "port":
                    LaunchConfiguration("port"),

                "mode":
                    LaunchConfiguration("mode"),

                "model":
                    LaunchConfiguration("model"),

                "gripper":
                    dsr_gripper,

                "mobile":
                    "none",

                "rt_host":
                    LaunchConfiguration("rt_host"),
            }],

            output="screen",
        )
    ]


# ================================================================
# robot_state_publisher
#
# RG2에서는:
#
# /dsr01/joint_states
#       +
# /gripper_joint_states
#       ↓
# /joint_states
#
# 를 사용
# ================================================================
def robot_state_publisher_fn(context):

    if is_rg2(context):

        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",

                name="robot_state_publisher",

                output="both",

                parameters=[{
                    "robot_description":
                        ParameterValue(
                            LaunchConfiguration(
                                "robot_description"
                            ),
                            value_type=str,
                        ),

                    "publish_frequency":
                        100.0,
                }],

                remappings=[
                    (
                        "joint_states",
                        "/joint_states",
                    ),
                ],
            )
        ]

    # 기존 Doosan 구성
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",

            name="robot_state_publisher",

            namespace=LaunchConfiguration(
                "name"
            ),

            output="both",

            parameters=[{
                "robot_description":
                    ParameterValue(
                        LaunchConfiguration(
                            "robot_description"
                        ),
                        value_type=str,
                    ),
            }],
        )
    ]


# ================================================================
# MoveGroup + RViz
# ================================================================
def rviz_and_move_group_fn(context):

    model_value = (
        LaunchConfiguration("model")
        .perform(context)
    )

    ns_value = robot_namespace(context)

    gripper_value = (
        LaunchConfiguration("gripper")
        .perform(context)
        .lower()
    )

    package_name = (
        f"dsr_moveit_config_{model_value}"
    )

    package_path = (
        FindPackageShare(package_name)
        .perform(context)
    )

    print(
        "MoveIt Config Package:",
        package_name,
    )

    print(
        "Package Path:",
        package_path,
    )

    # ------------------------------------------------------------
    # 중요
    #
    # 기존 dsr.srdf.xacro에는 RG2 semantic 정의가 없다.
    #
    # 우선 manipulator group은 기존 M0609 설정을 그대로
    # 사용하고 RG2 형상은 URDF collision model로 사용.
    #
    # rg2_tcp / end-effector 정의는 이후 SRDF에서 추가.
    # ------------------------------------------------------------
    semantic_gripper = gripper_value

    moveit_config = (
        MoveItConfigsBuilder(
            model_value,
            "robot_description",
            package_name,
        )

        .robot_description(
            file_path=(
                f"config/{model_value}"
                ".urdf.xacro"
            )
        )

        .robot_description_semantic(
            file_path=(
                "config/dsr.srdf.xacro"
            ),

            mappings={
                "gripper":
                    semantic_gripper
            },
        )

        .trajectory_execution(
            file_path=(
                "config/"
                "moveit_controllers.yaml"
            )
        )

        .planning_pipelines(
            pipelines=[
                "ompl",
                "chomp",
                "pilz_industrial_motion_planner",
            ],

            default_planning_pipeline=(
                "ompl"
            ),

            load_all=False,
        )

        # ------------------------------------------------------------
        # Octomap (occupancy map monitor).
        #
        # to_moveit_configs() 가 기본 경로를 자동으로 읽긴 하지만,
        # 빠지면 에러 없이 조용히 octomap 만 안 생긴다. 명시한다.
        # ------------------------------------------------------------
        .sensors_3d(
            file_path=(
                "config/sensors_3d.yaml"
            )
        )

        .to_moveit_configs()
    )

    # ------------------------------------------------------------
    # MoveIt 기본 M0609 URDF를
    # launch에서 생성한 robot_description으로 덮어쓴다.
    #
    # RG2 모드이면 여기 들어가는 것이:
    #
    # M0609
    # + RG2
    # + bracket
    # + D435
    #
    # 통합 URDF
    # ------------------------------------------------------------
    move_group_params = [

        moveit_config.to_dict(),

        {
            "robot_description":
                ParameterValue(
                    LaunchConfiguration(
                        "robot_description"
                    ),
                    value_type=str,
                )
        },
    ]

    rviz_params = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.robot_description_semantic,
    
        {
            "robot_description": ParameterValue(
                LaunchConfiguration(
                    "robot_description"
                ),
                value_type=str,
            )
        },
    
        # MoveIt RViz plugin이 /dsr01 + default_planning_pipeline
        # 형태로 파라미터를 찾기 때문에 명시적으로 제공
        {
            "default_planning_pipeline": "ompl",
            f"/{ns_value}default_planning_pipeline": "ompl",
        },
    ]
    # RG2에서는 합쳐진 root /joint_states 사용
    move_group_remappings = []

    if gripper_value == "rg2":

        move_group_remappings = [
            (
                "joint_states",
                "/joint_states",
            ),
        ]

    run_move_group_node = Node(

        package="moveit_ros_move_group",

        executable="move_group",

        namespace=LaunchConfiguration(
            "name"
        ),

        output="screen",

        parameters=move_group_params,

        remappings=move_group_remappings,
    )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_base = os.path.join(
        get_package_share_directory(
            package_name
        ),
        "launch",
    )

    rviz_full_config = os.path.join(
        rviz_base,
        "moveit.rviz",
    )

    with open(
        rviz_full_config,
        "r",
    ) as f:

        content = f.read()

    target_ns = (
        f"/{ns_value}"
        if ns_value
        else ""
    )

    new_content = re.sub(
        r"Move Group Namespace:.*",
        (
            f"Move Group Namespace: "
            f"{target_ns}"
        ),
        content,
    )

    tmp_rviz = (
        tempfile.NamedTemporaryFile(
            mode="w",
            delete=False,
            suffix=".rviz",
        )
    )

    tmp_rviz.write(
        new_content
    )

    tmp_rviz.close()

    tmp_rviz_path = (
        tmp_rviz.name
    )

    print(
        "[INFO] Generated dynamic "
        "RViz config:",
        tmp_rviz_path,
        "namespace:",
        target_ns,
    )

    rviz_node = Node(

        package="rviz2",

        executable="rviz2",

        name="rviz2",

        output="log",

        arguments=[
            "-d",
            tmp_rviz_path,
        ],

        parameters=rviz_params,
        
        remappings = [
            (
                "/monitored_planning_scene",
                f"/{ns_value}/monitored_planning_scene",
            ),
            (
                "/planning_scene",
                f"/{ns_value}/planning_scene",
            ),
            (
                "/planning_scene_world",
                f"/{ns_value}/planning_scene_world",
            ),
        ],

    )

    cleanup_rviz_file = (
        RegisterEventHandler(
            OnProcessExit(

                target_action=rviz_node,

                on_exit=[

                    LogInfo(
                        msg=(
                            ">> Cleaning RViz "
                            f"config: "
                            f"{tmp_rviz_path}"
                        )
                    ),

                    ExecuteProcess(
                        cmd=[
                            "rm",
                            tmp_rviz_path,
                        ],

                        output="screen",
                    ),
                ],
            )
        )
    )

    return [

        run_move_group_node,

        rviz_node,

        cleanup_rviz_file,
    ]


# ================================================================
# controller_manager
# ================================================================
def control_node_fn(context):

    params = [

        {
            "robot_description":
                ParameterValue(
                    LaunchConfiguration(
                        "robot_description"
                    ),
                    value_type=str,
                )
        },

        LaunchConfiguration(
            "controller_yaml"
        ),
    ]

    # 기존 Robotiq 지원 유지
    if (
        LaunchConfiguration("gripper")
        .perform(context)
        .lower()
        == "robotiq_2f85"
    ):

        pkg_share = (
            get_package_share_directory(
                "dsr_controller2"
            )
        )

        gripper_yaml = os.path.join(
            pkg_share,
            "config",
            "gripper_controller.yaml",
        )

        params.append(
            gripper_yaml
        )

        print(
            "[INFO] Including Robotiq YAML:",
            gripper_yaml,
        )

    node = Node(

        package="controller_manager",

        executable="ros2_control_node",

        namespace=LaunchConfiguration(
            "name"
        ),

        parameters=params,
        remappings=[
            (
                "robot_description",
                "/robot_description",
            ),
        ],
        output="both",
    )

    return [node]


# ================================================================
# 기존 Robotiq controller
# ================================================================
def gripper_spawner_fn(context):

    if (
        LaunchConfiguration("gripper")
        .perform(context)
        .lower()
        != "robotiq_2f85"
    ):
        return []

    return [
        Node(
            package="controller_manager",

            namespace=LaunchConfiguration(
                "name"
            ),

            executable="spawner",

            arguments=[
                "gripper_position_controller",
                "-c",
                "controller_manager",
            ],

            output="screen",
        )
    ]


# ================================================================
# RG2
#
# 기존 m0609_rg2_bringup에서 쓰던 구조를 그대로 가져옴.
# ================================================================
def rg2_nodes_fn(context):

    if not is_rg2(context):
        return []

    ns = robot_namespace(context)

    if not ns:

        raise RuntimeError(
            "RG2 사용 시 "
            "name:=dsr01 을 지정하세요."
        )

    mode = (
        LaunchConfiguration("mode")
        .perform(context)
        .lower()
    )

    nodes = []

    # ------------------------------------------------------------
    # REAL RG2
    # ------------------------------------------------------------
    if mode == "real":

        nodes.append(

            Node(
                package=(
                    "onrobot_rg_control"
                ),

                executable=(
                    "OnRobotRGControllerServer"
                ),

                name=(
                    "OnRobotRGControllerServer"
                ),

                output="screen",

                parameters=[{

                    "/onrobot/control":
                        "modbus",

                    "/onrobot/ip":
                        "192.168.1.1",

                    "/onrobot/port":
                        502,

                    "/onrobot/changer_addr":
                        65,

                    "/onrobot/gripper":
                        "rg2",

                    "/onrobot/offset":
                        5,
                }],

                remappings=[
                    (
                        "/joint_states",
                        "/onrobot_joint_states",
                    ),
                ],
            )
        )

    # ------------------------------------------------------------
    # VIRTUAL RG2
    # ------------------------------------------------------------
    else:

        nodes.append(

            Node(
                package=(
                    "m0609_rg2_bringup"
                ),

                executable=(
                    "gripper_virtual_node.py"
                ),

                name=(
                    "gripper_virtual_node"
                ),

                output="screen",
            )
        )

    # ------------------------------------------------------------
    # OnRobot joint 이름
    #       ↓
    # URDF rg2_* joint 이름
    # ------------------------------------------------------------
    nodes.append(

        Node(
            package=(
                "m0609_rg2_bringup"
            ),

            executable=(
                "gripper_joint_state_publisher.py"
            ),

            name=(
                "gripper_joint_state_publisher"
            ),

            output="screen",
        )
    )

    # ------------------------------------------------------------
    # M0609 + RG2 joint_states 합치기
    #
    # /dsr01/joint_states
    # /gripper_joint_states
    #          ↓
    # /joint_states
    # ------------------------------------------------------------
    nodes.append(

        Node(
            package=(
                "joint_state_publisher"
            ),

            executable=(
                "joint_state_publisher"
            ),

            name=(
                "joint_state_publisher"
            ),

            parameters=[{

                "source_list": [
                    f"/{ns}/joint_states",
                    "/gripper_joint_states",
                ],

                "rate":
                    100,
            }],

            output="screen",
        )
    )

    return nodes


# ================================================================
# Launch
# ================================================================
def generate_launch_description():

    ARGUMENTS = [

        DeclareLaunchArgument(
            "name",
            default_value="",
            description="NAME_SPACE",
        ),

        DeclareLaunchArgument(
            "host",
            default_value="127.0.0.1",
            description="ROBOT_IP",
        ),

        DeclareLaunchArgument(
            "port",
            default_value="12345",
            description="ROBOT_PORT",
        ),

        DeclareLaunchArgument(
            "mode",
            default_value="virtual",
            description="OPERATION MODE",
        ),

        DeclareLaunchArgument(
            "model",
            default_value="a0509",
            description="ROBOT_MODEL",
        ),

        DeclareLaunchArgument(
            "color",
            default_value="white",
            description="ROBOT_COLOR",
        ),

        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2",
        ),

        DeclareLaunchArgument(
            "gz",
            default_value="false",
            description="USE GAZEBO SIM",
        ),

        DeclareLaunchArgument(
            "rt_host",
            default_value="192.168.137.50",
            description="ROBOT_RT_IP",
        ),

        DeclareLaunchArgument(
            "dynamic_yaml",
            default_value="false",
            description="Use dynamic controller.yaml",
        ),

        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description=(
                "GRIPPER "
                "(none|robotiq_2f85|rg2)"
            ),
        ),
    ]

    # ------------------------------------------------------------
    # robot_description
    # ------------------------------------------------------------
    robot_description_action = (
        OpaqueFunction(
            function=(
                generate_robot_description_action
            )
        )
    )

    # ------------------------------------------------------------
    # DSR bringup
    # ------------------------------------------------------------
    run_emulator_node = (
        OpaqueFunction(
            function=run_emulator_fn
        )
    )

    # ------------------------------------------------------------
    # robot_state_publisher
    # ------------------------------------------------------------
    robot_state_pub_node = (
        OpaqueFunction(
            function=(
                robot_state_publisher_fn
            )
        )
    )

    # ------------------------------------------------------------
    # ros2_control
    # ------------------------------------------------------------
    control_node = (
        OpaqueFunction(
            function=control_node_fn
        )
    )

    # ------------------------------------------------------------
    # joint_state_broadcaster
    # ------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(

        package="controller_manager",

        namespace=LaunchConfiguration(
            "name"
        ),

        executable="spawner",

        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # ------------------------------------------------------------
    # dsr_controller2
    # ------------------------------------------------------------
    robot_controller_spawner = Node(

        package="controller_manager",

        namespace=LaunchConfiguration(
            "name"
        ),

        executable="spawner",

        arguments=[
            "dsr_controller2",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # ------------------------------------------------------------
    # dsr_moveit_controller
    # ------------------------------------------------------------
    dsr_moveit_controller_spawner = Node(

        package="controller_manager",

        executable="spawner",

        namespace=LaunchConfiguration(
            "name"
        ),

        arguments=[
            "dsr_moveit_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # ------------------------------------------------------------
    # MoveGroup + RViz
    # ------------------------------------------------------------
    rviz_and_move_group = (
        OpaqueFunction(
            function=rviz_and_move_group_fn
        )
    )

    # ------------------------------------------------------------
    # RG2
    # ------------------------------------------------------------
    rg2_nodes = (
        OpaqueFunction(
            function=rg2_nodes_fn
        )
    )

    # ============================================================
    # STEP 1
    #
    # joint_state_broadcaster
    #       ↓
    # dsr_controller2
    # ============================================================
    delay_robot_controller_after_joint_state = (
        RegisterEventHandler(

            OnProcessExit(

                target_action=(
                    joint_state_broadcaster_spawner
                ),

                on_exit=[

                    LogInfo(
                        msg=(
                            ">> [STEP 1] "
                            "joint_state_broadcaster "
                            "active. "
                            "Starting "
                            "dsr_controller2..."
                        )
                    ),

                    robot_controller_spawner,
                ],
            )
        )
    )

    # ============================================================
    # STEP 2
    #
    # 기존 Robotiq 전용
    # ============================================================
    delay_gripper_after_robot_controller = (
        RegisterEventHandler(

            OnProcessExit(

                target_action=(
                    robot_controller_spawner
                ),

                on_exit=[

                    LogInfo(
                        msg=(
                            ">> [STEP 2] "
                            "dsr_controller2 active. "
                            "Checking optional "
                            "Robotiq controller..."
                        )
                    ),

                    OpaqueFunction(
                        function=(
                            gripper_spawner_fn
                        )
                    ),
                ],
            )
        )
    )

    # ============================================================
    # STEP 3
    #
    # dsr_controller2
    #       ↓
    # dsr_moveit_controller
    # ============================================================
    delay_dsr_moveit_controller_after_robot_controller = (
        RegisterEventHandler(

            OnProcessExit(

                target_action=(
                    robot_controller_spawner
                ),

                on_exit=[

                    LogInfo(
                        msg=(
                            ">> [STEP 3] "
                            "dsr_controller2 active. "
                            "Starting "
                            "dsr_moveit_controller..."
                        )
                    ),

                    dsr_moveit_controller_spawner,
                ],
            )
        )
    )

    # ============================================================
    # STEP 4
    #
    # dsr_moveit_controller
    #       ↓
    # MoveGroup + RViz
    # ============================================================
    delay_rviz_after_moveit_controller = (
        RegisterEventHandler(

            OnProcessExit(

                target_action=(
                    dsr_moveit_controller_spawner
                ),

                on_exit=[

                    LogInfo(
                        msg=(
                            ">> [STEP 4] "
                            "dsr_moveit_controller "
                            "active. "
                            "Starting MoveGroup "
                            "and RViz..."
                        )
                    ),

                    rviz_and_move_group,
                ],
            )
        )
    )

    # ============================================================
    # Launch
    # ============================================================
    nodes = [

        LogInfo(
            msg=(
                ">> [START] "
                "Doosan M0609 + MoveIt2 "
                "bringup..."
            )
        ),

        robot_description_action,

        run_emulator_node,

        robot_state_pub_node,

        control_node,

        joint_state_broadcaster_spawner,

        # gripper:=rg2 일 때만 실제 노드 반환
        rg2_nodes,

        delay_robot_controller_after_joint_state,

        delay_gripper_after_robot_controller,

        delay_dsr_moveit_controller_after_robot_controller,

        delay_rviz_after_moveit_controller,
    ]

    return LaunchDescription(
        ARGUMENTS + nodes
    )
