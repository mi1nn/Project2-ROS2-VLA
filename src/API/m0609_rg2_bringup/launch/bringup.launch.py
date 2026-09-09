"""M0609 + RG2 그리퍼 + RealSense 브라켓 통합 bringup (ROS2 Jazzy).

기동하는 것:
  - [virtual] DRCF 에뮬레이터 (Docker 형제 컨테이너) + ros2_control 하드웨어 인터페이스
  - [real]    실 컨트롤러(host)에 연결하는 ros2_control 하드웨어 인터페이스
  - joint_state_broadcaster + dsr_controller2 (motion service — movej/movel 등)
  - 그리퍼: virtual = gripper_virtual_node(애니메이션) / real = OnRobot Modbus 드라이버
  - gripper_joint_state_publisher (두 모드 공통 — 드라이버 조인트명 → URDF 조인트명)
  - robot_state_publisher (팔 + 그리퍼 + 브라켓 + D435 통합 URDF) + RViz

사용 예:
  ros2 launch m0609_rg2_bringup bringup.launch.py                          # virtual (에뮬레이터)
  ros2 launch m0609_rg2_bringup bringup.launch.py camera:=true             # + RealSense 드라이버
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

기동 후 가상 pick & place:
  ros2 run m0609_rg2_bringup pick_place_demo.py
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 이 패키지는 팔 URDF·컨트롤러·에뮬레이터 러너를 전부 doosan-robot2 에서 가져온다. 없으면
# 기동이 xacro 안쪽의 PackageNotFoundError 로 터지는데, 그 트레이스백만 봐서는 무엇을 설치해야
# 하는지 알 수 없다. 그래서 launch 를 만들기 전에 먼저 확인하고 해법까지 같이 알려준다.
_REQUIRED_DSR_PACKAGES = {
    'dsr_description2': '팔 URDF + ros2_control 태그',
    'dsr_controller2':  '컨트롤러 설정',
    'dsr_bringup2':     'DRCF 에뮬레이터 러너 (mode:=virtual 에 필요)',
}


def _assert_dsr_available():
    """doosan-robot2 패키지가 검색 경로에 있는지 확인하고, 없으면 해법과 함께 실패한다."""
    missing = []
    for pkg, why in _REQUIRED_DSR_PACKAGES.items():
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(f'  - {pkg} ({why})')
    if not missing:
        return
    raise RuntimeError(
        'DSR 패키지를 찾을 수 없어 bringup 을 시작할 수 없습니다.\n'
        + '\n'.join(missing)
        + '\n\n이 패키지는 doosan-robot2(jazzy) 위에서만 동작합니다. 둘 중 하나로 해결하세요.\n'
        '  1) 이 워크스페이스에 직접 두기 — src 에 clone 후 다시 빌드:\n'
        '       git clone https://github.com/ROKEY-SPARK/doosan-robot2_jazzy.git src/doosan-robot2\n'
        '       colcon build --symlink-install\n'
        '  2) 이미 DSR 이 있는 워크스페이스를 겹쳐 쓰기 — 이 워크스페이스보다 먼저 source:\n'
        '       source ~/cobot_ws/install/setup.bash\n'
        '       source install/setup.bash\n'
    )


def generate_launch_description():
    _assert_dsr_available()

    args = [
        DeclareLaunchArgument(
            'mode', default_value='virtual', choices=['virtual', 'real'],
            description='virtual=DRCF 에뮬레이터(안전 기본) | real=실 컨트롤러 연결',
        ),
        DeclareLaunchArgument(
            'host', default_value='192.168.1.100',
            description='실기(mode:=real) 로봇 IP. virtual 이면 무시되고 127.0.0.1 로 강제',
        ),
        DeclareLaunchArgument('port', default_value='12345', description='DSR 컨트롤러 포트(DRFL)'),
        DeclareLaunchArgument(
            'rt_host', default_value='192.168.137.50',
            description='실기 RT control 채널 IP. dsr_bringup2 upstream 기본값과 동일',
        ),
        DeclareLaunchArgument(
            'camera', default_value='false', choices=['true', 'false'],
            description='RealSense 드라이버 기동 여부(mode 와 무관). 카메라를 따로 띄우려면 false',
        ),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 기동 여부'),
    ]

    is_real    = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' != 'real'"])

    # 안전 게이트: mode 가 정확히 'real' 일 때만 실기 IP 를 드라이버에 넘기고, 그 외는 전부
    # loopback(로컬 에뮬레이터)으로 강제한다. dsr_hardware2 는 받은 host 로 무조건 접속하고
    # mode 로 접속 대상을 바꾸지 않으므로, 실기 IP 가 virtual 로 새면 켜져 있는 실 로봇에 붙는다.
    robot_host = PythonExpression(
        ["'", LaunchConfiguration('host'),
         "' if '", LaunchConfiguration('mode'), "' == 'real' else '127.0.0.1'"]
    )

    # ── [virtual] DRCF 에뮬레이터 ────────────────────────────────────
    # run_drcf.sh 의 중복 컨테이너 체크는 'docker ps -q'(running 만) 기반이라 Exited 로 남은
    # --rm 미정리 컨테이너를 놓친다. 그러면 다음 bringup 의 'docker run --name dsr01_emulator'
    # 가 이름 충돌로 실패 → 에뮬레이터 미기동 → 하드웨어 init 실패로 연쇄. 기동 전에 동명
    # 컨테이너를 강제 제거해 launch 를 멱등하게 만든다.
    emulator_cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'docker rm -f dsr01_emulator 2>/dev/null || true'],
        condition=IfCondition(is_virtual),
        output='log',
    )
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace='dsr01',
        parameters=[
            {'name': 'dsr01'},
            {'host': robot_host},
            {'port': LaunchConfiguration('port')},
            {'mode': LaunchConfiguration('mode')},
            {'model': 'm0609'},
            {'gripper': 'none'},
            {'mobile': 'none'},
        ],
        condition=IfCondition(is_virtual),
        output='screen',
    )
    start_emulator = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=emulator_cleanup, on_exit=[run_emulator_node]),
    )

    # ── 통합 URDF (팔 + ros2_control + 그리퍼 + 브라켓 + D435) ────────
    # robot_state_publisher 와 controller_manager 가 같은 한 장을 본다. 통합 URDF 가
    # dsr_description2 의 xacro 를 include 해 <ros2_control> 태그까지 담고 있어 가능.
    xacro_file = os.path.join(
        get_package_share_directory('m0609_rg2_bringup'),
        'urdf', 'm0609_with_rg2_camera.urdf.xacro',
    )
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' host:=', robot_host,
            ' port:=', LaunchConfiguration('port'),
            # rt_host 를 빼면 xacro 기본값인 빈 문자열이 하드웨어 파라미터로 들어간다.
            # DRCF 3.0 이상에서 dsr_hardware2 가 이 값으로 RT control 채널을 여므로 실기에서 문제가 된다.
            ' rt_host:=', LaunchConfiguration('rt_host'),
            ' mode:=', LaunchConfiguration('mode'),
            ' model:=m0609',
            ' update_rate:=100',
        ]),
        value_type=str,
    )

    # Jazzy 의 controller_manager 는 robot_description 을 파라미터가 아니라 토픽에서 읽는다
    # (humble 은 파라미터). 네임스페이스가 dsr01 이라 기본 구독 대상이 /dsr01/robot_description
    # 인데 robot_state_publisher 는 루트에 있으므로, 루트 토픽으로 remap 한다.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='dsr01',
        parameters=[
            {'robot_description': robot_description},
            {'update_rate': 100},
            PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml']),
        ],
        remappings=[('robot_description', '/robot_description')],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    )
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='dsr01',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    # ── 그리퍼 ────────────────────────────────────────────────────────
    # virtual / real 모두 /onrobot_joint_states 를 내보내고, gripper_joint_state_publisher 가
    # 이를 URDF 조인트명(rg2_ prefix)으로 바꿔 /gripper_joint_states 로 재발행한다.
    # → 하위(joint_state_publisher → robot_state_publisher → RViz) 경로가 모드에 무관하게 동일.
    gripper_virtual_node = Node(
        package='m0609_rg2_bringup',
        executable='gripper_virtual_node.py',
        name='gripper_virtual_node',
        condition=IfCondition(is_virtual),
        output='screen',
    )
    onrobot_driver = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGControllerServer',
        name='OnRobotRGControllerServer',
        output='screen',
        parameters=[{
            '/onrobot/control':      'modbus',
            '/onrobot/ip':           '192.168.1.1',
            '/onrobot/port':         502,
            '/onrobot/changer_addr': 65,
            '/onrobot/gripper':      'rg2',
            '/onrobot/offset':       5,
        }],
        # 드라이버는 /joint_states 로 내보내 joint_state_publisher 출력과 충돌한다.
        remappings=[('/joint_states', '/onrobot_joint_states')],
        condition=IfCondition(is_real),
    )
    gripper_joint_state_publisher = Node(
        package='m0609_rg2_bringup',
        executable='gripper_joint_state_publisher.py',
        name='gripper_joint_state_publisher',
        output='screen',
    )

    # ── 통합 joint_states ─────────────────────────────────────────────
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/dsr01/joint_states', '/gripper_joint_states'],
            # 기본 10Hz 로 두면 robot_state_publisher 의 /tf 도 10Hz 가 되어,
            # 30Hz 포인트클라우드(stamp 지연 ~40ms)가 자기 stamp 시각의 TF 를
            # 못 받아 RViz message filter 에서 주기적으로 드롭된다(status
            # error/OK 반복). 원 소스 /dsr01/joint_states 가 100Hz 라 맞춘다.
            'rate': 100,
        }],
    )

    # ── robot_state_publisher ─────────────────────────────────────────
    # world → base_link 고정 조인트는 통합 URDF 안에 있으므로 별도 static_transform_publisher 불필요.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            # 기본 20Hz 스로틀. /joint_states 를 100Hz 로 넣어도 /tf 가 ~18Hz 로
            # 깎여, 30Hz 포인트클라우드(stamp 지연 ~40ms)의 stamp 시각 TF lookup 이
            # 실측 27% 실패했다(RViz PointCloud2 status error/OK 반복의 직접 원인).
            # 100 으로 올리면 /tf ~64Hz, 실패 0% (599 프레임 실측).
            'publish_frequency': 100.0,
        }],
    )

    # ── RealSense 드라이버 ────────────────────────────────────────────
    # mode 와 무관하게 camera 인자만 본다. 예전에는 mode:=real 이면 camera 값을 무시하고
    # 무조건 켰는데, 그러면 실기에서 카메라를 따로 띄워 보는 절차가 불가능하고
    # (launch 가 이미 USB 장치와 /camera 노드 이름을 점유한다) camera:=false 라는
    # 명시적 요청이 조용히 무시된다.
    camera_enabled = LaunchConfiguration('camera')
    # 토픽은 /camera/color/image_raw 형태로 나온다.
    #
    # realsense2_camera 는 스트림 토픽을 private('~/')으로 만들어(upstream src/rs_node_setup.cpp)
    # 최종 이름이 '/<node_namespace>/<node_name>/<stream>/...' 이 된다. 그리고 노드 자체의 기본
    # 네임스페이스가 생성자에 '/camera' 로 박혀 있다(upstream src/realsense_node_factory.cpp:34
    # RosNodeBase("camera", "/camera", ...)). 그래서 아무것도 안 주면 /camera/camera/... 라는
    # 중복 경로가 나오는데, 그 한 단계는 아무 정보도 담지 않는다.
    # namespace 인자를 생략하는 것으로는 안 되고(드라이버 기본값이 그대로 살아난다) 루트를
    # 명시해야 한 단계로 줄어든다.
    #
    # TF frame_id 는 이것과 무관하게 camera_name 파라미터(기본 'camera')에서 나오므로
    # URDF 의 camera_link / camera_*_optical_frame 은 그대로다. 노드 이름과 camera_name 을
    # 다르게 주면 토픽과 프레임이 조용히 갈라지니 함께 바꿀 것.
    #
    # 프로파일 값은 cobot2_bringup 에서 실측 검증된 조합을 그대로 가져왔다.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='/',
        name='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '1280x720x30',
            'align_depth.enable': True,
            'enable_rgbd': True,
            'enable_sync': True,
            'pointcloud.enable': True,
            # 2 = color. 미지정 시 0(ANY)으로 떠서 텍스처 매칭이 실패하고(경고
            # "No matching stream for texture") cloud 가 rgb 없이 발행된다 —
            # RViz 기본 설정(RGB8 transformer)은 rgb 를 요구하므로 표시가 깨진다.
            'pointcloud.stream_filter': 2,
            # rs_launch.py 는 IMU 를 기본 off 로 두지만 launch 를 우회하면 켜진 채
            # 뜨고, D435i 에서 "Motion Module failure" 하드웨어 에러를 뱉는다.
            # 이 파이프라인은 IMU 를 쓰지 않는다.
            'enable_accel': False,
            'enable_gyro': False,
            'initial_reset': True,
        }],
        condition=IfCondition(camera_enabled),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(
            get_package_share_directory('m0609_rg2_bringup'), 'rviz', 'default.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [
        emulator_cleanup,
        start_emulator,
        control_node,
        joint_state_broadcaster_spawner,
        delay_controller,
        gripper_virtual_node,
        onrobot_driver,
        gripper_joint_state_publisher,
        joint_state_publisher_node,
        robot_state_publisher,
        realsense_node,
        rviz_node,
    ])
