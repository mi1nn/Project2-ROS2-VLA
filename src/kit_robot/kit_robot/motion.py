import json
import math
import os
import threading
import time
import warnings

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker

from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .onrobot import RG


def merge_octomap_acm(entry_names, matrix, allowed_links):
    """Return (names, matrix) with "<octomap>" allowed against allowed_links.

    Pure list surgery so it can be checked without a running move_group. The
    input must be the *current* ACM: a PlanningScene diff carrying a non-empty
    ACM replaces the whole matrix, so anything dropped here (the SRDF
    self-collision pairs) stops being ignored and planning dies on self-hits.
    """
    names = list(entry_names)
    matrix = [list(row) for row in matrix]
    for name in ["<octomap>"] + list(allowed_links):
        if name in names:
            continue
        names.append(name)
        for row in matrix:
            row.append(False)
        matrix.append([False] * len(names))
    octomap_index = names.index("<octomap>")
    for name in allowed_links:
        index = names.index(name)
        matrix[octomap_index][index] = True
        matrix[index][octomap_index] = True
    return (names, matrix)


class Motion:
    """MoveIt2 based motion backend.

    Public methods intentionally keep the same contract as the previous DSR_ROBOT2
    implementation so Controller and PositionEstimationNode do not need to know which
    robot motion backend is in use.

    External pose convention kept for compatibility:
      [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    where orientation is ZYZ Euler, matching the previous Doosan posx convention used
    by position_estimation.py and the hand-eye calibration data.
    """

    def __init__(self, node):
        if node is None:
            raise ValueError("ROS 2 node is required.")

        self.node = node
        self.logger = node.get_logger()

        config_path = os.path.join(
            get_package_share_directory("kit_robot"), "config", "motion.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)["motion"]

        self.positions = config["positions"]
        self.place_config = config["place"]
        self.place_slots = self.place_config["slots"]
        self.moveit_config = config.get("moveit", {})

        live_gg = config.get("graspgenx_live", {})
        self.cup_grasp_component = str(live_gg.get("component_name", "컵라면"))
        self.cup_grasp_service = str(live_gg.get("service", "/cup_pick/perception"))
        self.cup_grasp_snapshot = os.path.expanduser(
            str(live_gg.get("snapshot", "/tmp/graspgenx_live/latest.npz"))
        )
        self.cup_camera_frame = str(
            live_gg.get("camera_frame", "camera_color_optical_frame")
        )
        self.cup_tool_frame = str(live_gg.get("tool_frame", "tool0"))
        self.cup_grasp_top_k = int(live_gg.get("top_k", 5))
        self.cup_grasp_max_tilt_deg = float(live_gg.get("max_tilt_deg", 30.0))
        self.cup_grasp_offset_mm = float(live_gg.get("grasp_offset_mm", 20.0))
        self.cup_grasp_pregrasp_distance_mm = float(
            live_gg.get("pregrasp_distance_mm", 40.0)
        )
        self.cup_grasp_linear_vel_mm_s = float(live_gg.get("linear_vel_mm_s", 50.0))
        self.cup_grasp_linear_acc_mm_s2 = float(live_gg.get("linear_acc_mm_s2", 100.0))
        self.cup_grasp_perception_timeout_sec = float(
            live_gg.get("perception_timeout_sec", 180.0)
        )
        self.cup_grasp_snapshot_timeout_sec = float(
            live_gg.get("snapshot_timeout_sec", 3.0)
        )
        if self.cup_grasp_top_k < 1:
            raise ValueError("graspgenx_live.top_k must be >= 1")
        if not 0.0 < self.cup_grasp_max_tilt_deg <= 90.0:
            raise ValueError("graspgenx_live.max_tilt_deg must be in (0, 90]")
        if self.cup_grasp_offset_mm < 0.0:
            raise ValueError("graspgenx_live.grasp_offset_mm must be >= 0")
        if self.cup_grasp_pregrasp_distance_mm <= 0.0:
            raise ValueError("graspgenx_live.pregrasp_distance_mm must be > 0")

        default_grasp_to_tool = [
            0.0,
            0.0,
            -1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            -0.004,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.cup_grasp_to_tool = np.asarray(
            live_gg.get("grasp_to_tool_matrix", default_grasp_to_tool),
            dtype=np.float64,
        ).reshape(4, 4)
        if not np.all(np.isfinite(self.cup_grasp_to_tool)):
            raise ValueError("graspgenx_live.grasp_to_tool_matrix must be finite")
        if not np.allclose(self.cup_grasp_to_tool[3], [0.0, 0.0, 0.0, 1.0]):
            raise ValueError("graspgenx_live.grasp_to_tool_matrix must be homogeneous")
        grasp_to_tool_rotation = self.cup_grasp_to_tool[:3, :3]
        if not np.allclose(
            grasp_to_tool_rotation.T @ grasp_to_tool_rotation,
            np.eye(3),
            atol=1e-6,
        ) or not np.isclose(np.linalg.det(grasp_to_tool_rotation), 1.0, atol=1e-6):
            raise ValueError("graspgenx_live.grasp_to_tool_matrix rotation is invalid")
        grasp_params_path = os.path.join(
            get_package_share_directory("kit_robot"), "resource", "grasp_params.json"
        )
        with open(grasp_params_path, "r", encoding="utf-8") as file:
            self.grasp_params = json.load(file)

        self.group_name = self.moveit_config.get("planning_group", "manipulator")
        self.base_frame = self.moveit_config.get("base_frame", "base_link")
        self.eef_link = self.moveit_config.get("eef_link", "tool0")
        self.joint_names = list(
            self.moveit_config.get(
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            )
        )
        if len(self.joint_names) != 6:
            raise ValueError("moveit.joint_names must contain exactly 6 joints.")

        self.move_group_action_name = self.moveit_config.get(
            "move_group_action", "/dsr01/move_action"
        )
        self.execute_action_name = self.moveit_config.get(
            "execute_trajectory_action", "/execute_trajectory"
        )
        self.cartesian_service_name = self.moveit_config.get(
            "cartesian_path_service", "/compute_cartesian_path"
        )
        self.apply_planning_scene_service_name = self.moveit_config.get(
            "apply_planning_scene_service", "/dsr01/apply_planning_scene"
        )

        self.keepout_config = self.moveit_config.get("default_keepout_box", {})
        self.keepout_enabled = bool(self.keepout_config.get("enabled", True))
        self.keepout_object_id = str(self.keepout_config.get("id", "base_keepout_box"))
        self.keepout_min_cm = list(
            self.keepout_config.get("min_cm", [-200.0, 40.0, -100.0])
        )
        self.keepout_max_cm = list(
            self.keepout_config.get("max_cm", [200.0, 100.0, 600.0])
        )
        self.keepout_marker_topic = str(
            self.keepout_config.get(
                "marker_topic", "/kit_robot/default_keepout_box_marker"
            )
        )

        self.octomap_config = self.moveit_config.get("octomap", {})
        self.octomap_enabled = bool(self.octomap_config.get("enabled", True))
        self.octomap_cloud_in = str(
            self.octomap_config.get("cloud_in", "/camera/depth/color/points")
        )
        self.octomap_cloud_out = str(
            self.octomap_config.get("cloud_out", "/kit/octomap_cloud")
        )
        self.octomap_clear_service = str(
            self.octomap_config.get("clear_service", "/dsr01/clear_octomap")
        )
        self.octomap_get_scene_service = str(
            self.octomap_config.get(
                "get_planning_scene_service", "/dsr01/get_planning_scene"
            )
        )

        self.static_octomap_scan_sec = float(
            self.octomap_config.get("static_scan_sec", 3.0)
        )
        self.static_octomap_voxel_size_m = float(
            self.octomap_config.get("static_voxel_size_m", 0.01)
        )
        self.static_octomap_min_depth_m = float(
            self.octomap_config.get("static_min_depth_m", 0.28)
        )
        self.static_octomap_neighbor_min_count = int(
            self.octomap_config.get("static_neighbor_min_count", 4)
        )
        if self.static_octomap_scan_sec <= 0.0:
            raise ValueError("octomap.static_scan_sec must be > 0")
        if self.static_octomap_voxel_size_m <= 0.0:
            raise ValueError("octomap.static_voxel_size_m must be > 0")
        if self.static_octomap_min_depth_m < 0.0:
            raise ValueError("octomap.static_min_depth_m must be >= 0")
        if not 1 <= self.static_octomap_neighbor_min_count <= 27:
            raise ValueError("octomap.static_neighbor_min_count must be in [1, 27]")
        self.octomap_allowed_links = [
            str(name)
            for name in self.octomap_config.get(
                "allowed_collision_links", ["link_6", "tool0", "rg2_base_link"]
            )
        ]

        self.server_timeout = float(
            self.moveit_config.get("server_ready_timeout_sec", 15.0)
        )
        self.motion_timeout = float(self.moveit_config.get("motion_timeout_sec", 60.0))
        self.tf_timeout = float(self.moveit_config.get("tf_timeout_sec", 2.0))
        self.planning_time = float(self.moveit_config.get("planning_time_sec", 5.0))
        self.planning_attempts = int(self.moveit_config.get("planning_attempts", 5))
        self.pipeline_id = str(self.moveit_config.get("pipeline_id", ""))
        self.planner_id = str(self.moveit_config.get("planner_id", ""))

        self.joint_tolerance_rad = math.radians(
            float(self.moveit_config.get("joint_tolerance_deg", 0.5))
        )
        self.default_joint_velocity_scale = self._clamp_scale(
            self.moveit_config.get("default_joint_velocity_scale", 0.15)
        )
        self.default_joint_acceleration_scale = self._clamp_scale(
            self.moveit_config.get("default_joint_acceleration_scale", 0.15)
        )

        self.cartesian_max_step_m = float(
            self.moveit_config.get("cartesian_max_step_m", 0.005)
        )
        self.cartesian_min_fraction = float(
            self.moveit_config.get("cartesian_min_fraction", 0.999)
        )
        self.cartesian_jump_threshold = float(
            self.moveit_config.get("cartesian_jump_threshold", 0.0)
        )
        self.cartesian_prismatic_jump_threshold = float(
            self.moveit_config.get("cartesian_prismatic_jump_threshold", 0.0)
        )
        self.cartesian_revolute_jump_threshold = float(
            self.moveit_config.get("cartesian_revolute_jump_threshold", 0.0)
        )
        self.cartesian_acc_reference_mm_s2 = float(
            self.moveit_config.get("cartesian_acc_reference_mm_s2", 1000.0)
        )
        if self.cartesian_max_step_m <= 0.0:
            raise ValueError("moveit.cartesian_max_step_m must be > 0.")
        if not 0.0 < self.cartesian_min_fraction <= 1.0:
            raise ValueError("moveit.cartesian_min_fraction must be in (0, 1].")
        if self.cartesian_acc_reference_mm_s2 <= 0.0:
            raise ValueError("moveit.cartesian_acc_reference_mm_s2 must be > 0.")

        self._moveit_node = rclpy.create_node("kit_moveit_interface")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._moveit_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="kit_moveit_executor", daemon=True
        )
        self._spin_thread.start()

        self._move_group_client = ActionClient(
            self._moveit_node, MoveGroup, self.move_group_action_name
        )
        self._execute_client = ActionClient(
            self._moveit_node, ExecuteTrajectory, self.execute_action_name
        )
        self._cartesian_client = self._moveit_node.create_client(
            GetCartesianPath, self.cartesian_service_name
        )
        self._cup_perception_client = self._moveit_node.create_client(
            Trigger, self.cup_grasp_service
        )
        self._apply_planning_scene_client = self._moveit_node.create_client(
            ApplyPlanningScene, self.apply_planning_scene_service_name
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._keepout_marker_pub = self._moveit_node.create_publisher(
            Marker, self.keepout_marker_topic, marker_qos
        )

        self._octomap_mapping = False
        self.static_octomap_initialized = False
        self._static_octomap_frozen = False
        self._static_octomap_frames = 0
        self._static_octomap_input_points = 0
        self._static_octomap_invalid_or_near_dropped = 0
        self._static_octomap_voxels_before_density = 0
        self._static_octomap_density_dropped = 0
        self._static_octomap_output_voxels = 0

        self._clear_octomap_client = None
        self._get_planning_scene_client = None
        if self.octomap_enabled:
            self._clear_octomap_client = self._moveit_node.create_client(
                Empty, self.octomap_clear_service
            )
            self._get_planning_scene_client = self._moveit_node.create_client(
                GetPlanningScene, self.octomap_get_scene_service
            )
            self._octomap_cloud_pub = self._moveit_node.create_publisher(
                PointCloud2, self.octomap_cloud_out, qos_profile_sensor_data
            )
            self._moveit_node.create_subscription(
                PointCloud2,
                self.octomap_cloud_in,
                self._octomap_cloud_callback,
                qos_profile_sensor_data,
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._moveit_node, spin_thread=False
        )

        self.rg = RG("rg2", "192.168.1.1", 502)

        self._wait_for_moveit_servers()
        self._add_default_keepout_box()
        self._allow_octomap_collisions()
        self.logger.info(
            f"MoveIt2 Motion initialized: group={self.group_name}, base={self.base_frame}, eef={self.eef_link}"
        )

    @staticmethod
    def _clamp_scale(value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("MoveIt scaling factor must be finite.")
        return min(1.0, max(0.001, value))

    def _wait_for_moveit_servers(self):
        if not self._move_group_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"MoveGroup action unavailable: {self.move_group_action_name}"
            )
        if not self._execute_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"ExecuteTrajectory action unavailable: {self.execute_action_name}"
            )
        if not self._cartesian_client.wait_for_service(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"GetCartesianPath service unavailable: {self.cartesian_service_name}"
            )
        needs_planning_scene = self.keepout_enabled or self.octomap_enabled
        if needs_planning_scene and (
            not self._apply_planning_scene_client.wait_for_service(
                timeout_sec=self.server_timeout
            )
        ):
            raise RuntimeError(
                f"ApplyPlanningScene service unavailable: {self.apply_planning_scene_service_name}"
            )
        for client, name in (
            (self._clear_octomap_client, self.octomap_clear_service),
            (self._get_planning_scene_client, self.octomap_get_scene_service),
        ):
            if not self.octomap_enabled:
                break
            if client.wait_for_service(timeout_sec=self.server_timeout):
                continue
            self.logger.warning(
                f"Octomap service unavailable: {name} — octomap disabled. Obstacle avoidance falls back to the keepout box only."
            )
            self.octomap_enabled = False
            self._octomap_mapping = False

    def _wait_future(self, future, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and (not future.done()):
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{description} timed out after {timeout_sec:.1f}s")
            time.sleep(0.01)
        if not rclpy.ok() and (not future.done()):
            raise RuntimeError(f"ROS shutdown while waiting for {description}")
        if future.cancelled():
            raise RuntimeError(f"{description} was cancelled")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{description} failed: {exception}") from exception
        return future.result()

    @staticmethod
    def _error_name(code):
        return f"MoveItErrorCodes({code})"

    @staticmethod
    def _pose6_to_ros_pose(pose6):
        pose = list(pose6)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")
        if not all((math.isfinite(float(v)) for v in pose)):
            raise ValueError("target_pose contains non-finite values")
        msg = Pose()
        msg.position.x = float(pose[0]) / 1000.0
        msg.position.y = float(pose[1]) / 1000.0
        msg.position.z = float(pose[2]) / 1000.0
        quat = Rotation.from_euler(
            "ZYZ", [float(pose[3]), float(pose[4]), float(pose[5])], degrees=True
        ).as_quat()
        msg.orientation.x = float(quat[0])
        msg.orientation.y = float(quat[1])
        msg.orientation.z = float(quat[2])
        msg.orientation.w = float(quat[3])
        return msg

    def _joint_scales(self, config):
        vel_scale = config.get(
            "joint_velocity_scale", self.default_joint_velocity_scale
        )
        acc_scale = config.get(
            "joint_acceleration_scale", self.default_joint_acceleration_scale
        )
        return (self._clamp_scale(vel_scale), self._clamp_scale(acc_scale))

    @staticmethod
    def _validate_keepout_bounds(min_cm, max_cm):
        if len(min_cm) != 3 or len(max_cm) != 3:
            raise ValueError("default_keepout_box min_cm/max_cm must contain [x, y, z]")
        min_values = [float(v) for v in min_cm]
        max_values = [float(v) for v in max_cm]
        if not all((math.isfinite(v) for v in min_values + max_values)):
            raise ValueError("default_keepout_box contains non-finite values")
        for axis, low, high in zip("xyz", min_values, max_values):
            if high <= low:
                raise ValueError(
                    f"default_keepout_box max_{axis} must be greater than min_{axis}"
                )
        return (min_values, max_values)

    def _keepout_geometry_m(self):
        min_cm, max_cm = self._validate_keepout_bounds(
            self.keepout_min_cm, self.keepout_max_cm
        )
        center_m = [(low + high) * 0.5 / 100.0 for low, high in zip(min_cm, max_cm)]
        size_m = [(high - low) / 100.0 for low, high in zip(min_cm, max_cm)]
        return (min_cm, max_cm, center_m, size_m)

    def _publish_keepout_marker(self, center_m, size_m):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self._moveit_node.get_clock().now().to_msg()
        marker.ns = "kit_robot_keepout"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center_m[0]
        marker.pose.position.y = center_m[1]
        marker.pose.position.z = center_m[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = size_m[0]
        marker.scale.y = size_m[1]
        marker.scale.z = size_m[2]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self._keepout_marker_pub.publish(marker)

    def _add_default_keepout_box(self):
        if not self.keepout_enabled:
            self.logger.info("Default keepout box disabled")
            return False
        min_cm, max_cm, center_m, size_m = self._keepout_geometry_m()
        collision = CollisionObject()
        collision.header.frame_id = self.base_frame
        collision.header.stamp = self._moveit_node.get_clock().now().to_msg()
        collision.id = self.keepout_object_id
        collision.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size_m)
        box_pose = Pose()
        box_pose.position.x = center_m[0]
        box_pose.position.y = center_m[1]
        box_pose.position.z = center_m[2]
        box_pose.orientation.w = 1.0
        collision.primitives = [box]
        collision.primitive_poses = [box_pose]
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_planning_scene_client.call_async(request)
        response = self._wait_future(
            future, self.server_timeout, "ApplyPlanningScene(default_keepout_box)"
        )
        if response is None or not response.success:
            raise RuntimeError("Failed to add default keepout box")
        self._publish_keepout_marker(center_m, size_m)
        center_cm = [value * 100.0 for value in center_m]
        size_cm = [value * 100.0 for value in size_m]
        self.logger.info(
            f"Default keepout box added: id={self.keepout_object_id}, frame={self.base_frame}, min_cm={min_cm}, max_cm={max_cm}, center_cm={[round(v, 3) for v in center_cm]}, size_cm={[round(v, 3) for v in size_cm]}, marker_topic={self.keepout_marker_topic}"
        )
        return True

    def _octomap_cloud_callback(self, msg):
        if not self._octomap_mapping:
            return
        filtered = self._filter_static_octomap_cloud(msg)
        if filtered is None or filtered.width == 0:
            return
        self._octomap_cloud_pub.publish(filtered)

    @staticmethod
    def _cloud_from_selected_rows(msg, row_indices):
        if msg.point_step <= 0:
            return None
        n_points = len(msg.data) // msg.point_step
        if n_points == 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)
        row_indices = np.asarray(row_indices, dtype=np.int64)
        row_indices = row_indices[(row_indices >= 0) & (row_indices < n_points)]
        filtered = PointCloud2()
        filtered.header = msg.header
        filtered.height = 1
        filtered.width = int(len(row_indices))
        filtered.fields = msg.fields
        filtered.is_bigendian = msg.is_bigendian
        filtered.point_step = msg.point_step
        filtered.row_step = msg.point_step * filtered.width
        filtered.is_dense = True
        filtered.data = raw[row_indices].tobytes()
        return filtered

    def _filter_static_octomap_cloud(self, msg):
        """Prepare a stable cloud for the fixed OctoMap.

        Pipeline:
          raw D435i PointCloud2
            -> finite XYZ only
            -> reject optical depth < 0.28 m
            -> transform points to base_link
            -> 1 cm voxel downsample
            -> 3x3x3 occupied-neighborhood filter
            -> publish one representative point per surviving voxel

        The 3x3x3 test uses a Chebyshev distance of one voxel.  With the default
        threshold 4/27, isolated depth speckles disappear while one-voxel-thick
        box walls and corners normally remain connected.

        Output points are kept in the original camera frame. MoveIt's updater
        performs its normal TF and ray insertion after receiving this message.
        """
        if msg.point_step <= 0:
            return None
        offsets = {field.name: int(field.offset) for field in msg.fields}
        if not {"x", "y", "z"} <= offsets.keys():
            self.logger.warning(
                "[STATIC OCTOMAP] PointCloud2 has no XYZ fields; frame dropped"
            )
            return None
        if max(offsets["x"], offsets["y"], offsets["z"]) + 4 > msg.point_step:
            self.logger.warning(
                "[STATIC OCTOMAP] invalid XYZ field offsets; frame dropped"
            )
            return None
        n_points = len(msg.data) // msg.point_step
        if n_points == 0:
            return None
        dtype = np.dtype(">f4") if msg.is_bigendian else np.dtype("<f4")
        try:
            x = np.ndarray(
                (n_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=offsets["x"],
                strides=(msg.point_step,),
            )
            y = np.ndarray(
                (n_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=offsets["y"],
                strides=(msg.point_step,),
            )
            z = np.ndarray(
                (n_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=offsets["z"],
                strides=(msg.point_step,),
            )
        except Exception as error:
            self.logger.warning(f"[STATIC OCTOMAP] XYZ extraction failed: {error}")
            return None
        xyz_camera = np.column_stack((x, y, z)).astype(np.float64, copy=False)
        finite = np.all(np.isfinite(xyz_camera), axis=1)
        usable = finite & (xyz_camera[:, 2] > self.static_octomap_min_depth_m)
        source_indices = np.flatnonzero(usable)
        if len(source_indices) == 0:
            self._static_octomap_frames += 1
            self._static_octomap_input_points += int(n_points)
            self._static_octomap_invalid_or_near_dropped += int(n_points)
            return None
        xyz_camera = xyz_camera[source_indices]
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:
            self.logger.warning(
                f"[STATIC OCTOMAP] TF unavailable; frame dropped: {self.base_frame} <- {msg.header.frame_id}: {error}"
            )
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        R_base_cloud = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)
        xyz_base = (R_base_cloud @ xyz_camera.T).T + trans
        voxel_size = self.static_octomap_voxel_size_m
        voxel_indices = np.floor(xyz_base / voxel_size).astype(np.int64)
        unique_voxels, first_local = np.unique(voxel_indices, axis=0, return_index=True)
        representative_rows = source_indices[first_local]
        if len(unique_voxels) == 0:
            return None
        tree = cKDTree(unique_voxels.astype(np.float64))
        neighbor_counts = tree.query_ball_point(
            unique_voxels.astype(np.float64),
            r=1.0 + 1e-09,
            p=np.inf,
            return_length=True,
        )
        neighbor_counts = np.asarray(neighbor_counts, dtype=np.int32)
        density_ok = neighbor_counts >= self.static_octomap_neighbor_min_count
        output_rows = representative_rows[density_ok]
        self._static_octomap_frames += 1
        self._static_octomap_input_points += int(n_points)
        self._static_octomap_invalid_or_near_dropped += int(
            n_points - len(source_indices)
        )
        self._static_octomap_voxels_before_density += int(len(unique_voxels))
        self._static_octomap_density_dropped += int(
            len(unique_voxels) - np.count_nonzero(density_ok)
        )
        self._static_octomap_output_voxels += int(len(output_rows))
        return self._cloud_from_selected_rows(msg, output_rows)

    def set_octomap_mapping(self, enabled, *, force=False):
        if not self.octomap_enabled:
            return False
        enabled = bool(enabled)
        if enabled and self._static_octomap_frozen and (not force):
            self._octomap_mapping = False
            return False
        self._octomap_mapping = enabled
        return enabled

    def clear_octomap(self, *, force=False):
        if not self.octomap_enabled:
            return False
        if self._static_octomap_frozen and (not force):
            return False
        future = self._clear_octomap_client.call_async(Empty.Request())
        try:
            self._wait_future(future, self.server_timeout, "ClearOctomap")
        except Exception as error:
            self.logger.warning(f"ClearOctomap failed: {error}")
            return False
        self.logger.info("Octomap cleared")
        return True

    def start_static_octomap_scan(self):
        if not self.octomap_enabled:
            self.static_octomap_initialized = True
            self._static_octomap_frozen = True
            self.logger.warning("Octomap disabled; static-map scan skipped")
            return False
        if self.static_octomap_initialized:
            self.logger.info("Static OctoMap already initialized; scan skipped")
            return False
        self.set_octomap_mapping(False, force=True)
        if not self.clear_octomap(force=True):
            raise RuntimeError("Static OctoMap scan could not clear the previous map")
        self._move_named_position("inspection_pose")
        self._static_octomap_frames = 0
        self._static_octomap_input_points = 0
        self._static_octomap_invalid_or_near_dropped = 0
        self._static_octomap_voxels_before_density = 0
        self._static_octomap_density_dropped = 0
        self._static_octomap_output_voxels = 0
        self.set_octomap_mapping(True, force=True)
        self.logger.info(
            f"[STATIC OCTOMAP] START scan={self.static_octomap_scan_sec:.1f}s, voxel={self.static_octomap_voxel_size_m * 100.0:.1f}cm, min_depth={self.static_octomap_min_depth_m * 100.0:.0f}cm, neighborhood=3x3x3, min_occupied={self.static_octomap_neighbor_min_count}/27"
        )
        return True

    def finish_static_octomap_scan(self):
        if self.static_octomap_initialized:
            return True
        if not self.octomap_enabled:
            self.static_octomap_initialized = True
            self._static_octomap_frozen = True
            return True
        self.set_octomap_mapping(False, force=True)
        if self._static_octomap_frames == 0:
            raise RuntimeError(
                "Static OctoMap scan received no valid PointCloud2 frame"
            )
        self.static_octomap_initialized = True
        self._static_octomap_frozen = True
        self.logger.info(
            f"[STATIC OCTOMAP] FROZEN | frames={self._static_octomap_frames}, raw_points={self._static_octomap_input_points}, invalid_or_depth_lt_28cm_dropped={self._static_octomap_invalid_or_near_dropped}, voxels_before_density={self._static_octomap_voxels_before_density}, density_dropped={self._static_octomap_density_dropped}, published_voxels={self._static_octomap_output_voxels}"
        )
        return True

    def _allow_octomap_collisions(self):
        if not self.octomap_enabled or not self.octomap_allowed_links:
            return False
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._get_planning_scene_client.call_async(request)
        response = self._wait_future(
            future, self.server_timeout, "GetPlanningScene(ACM)"
        )
        acm = response.scene.allowed_collision_matrix
        if not acm.entry_names:
            raise RuntimeError(
                "Planning scene returned an empty ACM; refusing to overwrite self-collision pairs"
            )
        unknown = [
            name for name in self.octomap_allowed_links if name not in acm.entry_names
        ]
        if unknown:
            self.logger.warning(
                f"octomap.allowed_collision_links not in the robot ACM: {unknown}"
            )
        allowed_links = [
            name for name in self.octomap_allowed_links if name in acm.entry_names
        ]
        if not allowed_links:
            self.logger.warning(
                "No configured OctoMap collision links exist in the ACM"
            )
            return False
        names, matrix = merge_octomap_acm(
            list(acm.entry_names),
            [list(entry.enabled) for entry in acm.entry_values],
            allowed_links,
        )
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix.entry_names = names
        scene.allowed_collision_matrix.entry_values = [
            AllowedCollisionEntry(enabled=row) for row in matrix
        ]
        scene.allowed_collision_matrix.default_entry_names = list(
            acm.default_entry_names
        )
        scene.allowed_collision_matrix.default_entry_values = list(
            acm.default_entry_values
        )
        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_future = self._apply_planning_scene_client.call_async(apply_request)
        apply_response = self._wait_future(
            apply_future, self.server_timeout, "ApplyPlanningScene(octomap ACM)"
        )
        if apply_response is None or not apply_response.success:
            raise RuntimeError("Failed to allow octomap collisions for the gripper")
        self.logger.info(
            f"Octomap collisions allowed for {len(allowed_links)} links: {allowed_links}"
        )
        return True

    def move_joint(self, joint_deg, velocity_scale=None, acceleration_scale=None):
        self.set_octomap_mapping(False)
        values = list(joint_deg)
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"joint target must have {len(self.joint_names)} values, got {len(values)}"
            )
        if not all((math.isfinite(float(v)) for v in values)):
            raise ValueError("joint target contains non-finite values")
        vel_scale = self._clamp_scale(
            self.default_joint_velocity_scale
            if velocity_scale is None
            else velocity_scale
        )
        acc_scale = self._clamp_scale(
            self.default_joint_acceleration_scale
            if acceleration_scale is None
            else acceleration_scale
        )
        constraints = Constraints()
        constraints.name = "kit_robot_joint_goal"
        for joint_name, value_deg in zip(self.joint_names, values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = math.radians(float(value_deg))
            joint_constraint.tolerance_above = self.joint_tolerance_rad
            joint_constraint.tolerance_below = self.joint_tolerance_rad
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True
        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.logger.info(
            f"MoveIt joint goal: {values}, vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f}"
        )
        send_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future, self.server_timeout, "sending MoveGroup goal"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected")
        result_future = goal_handle.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future, self.motion_timeout, "MoveGroup execution"
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise
        result = wrapped_result.result
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "MoveGroup plan/execute failed: " + self._error_name(code)
            )
        return 0

    def _move_named_position(self, name):
        """Move to a named position from motion.yaml.

        Supported types:
          - joint: [j1, j2, j3, j4, j5, j6] in degrees
          - pose/cartesian: [x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]

        The cartesian alias is retained for YAML compatibility, but it uses
        MoveGroup pose-goal planning rather than a Cartesian straight path.
        """
        if name not in self.positions:
            raise KeyError(f"Unknown named position: {name}")
        config = self.positions[name]
        pose_type = str(config.get("type", "")).strip().lower()
        target = list(config.get("pos", []))
        vel_scale, acc_scale = self._joint_scales(config)
        if pose_type == "joint":
            if len(target) != len(self.joint_names):
                raise ValueError(
                    f"{name}.pos must contain {len(self.joint_names)} joint values"
                )
            return self.move_joint(
                target, velocity_scale=vel_scale, acceleration_scale=acc_scale
            )
        if pose_type in ("cartesian", "pose"):
            if len(target) != 6:
                raise ValueError(
                    f"{name}.pos must be [x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]"
                )
            position_tolerance_mm = float(config.get("position_tolerance_mm", 2.0))
            orientation_tolerance_deg = float(
                config.get("orientation_tolerance_deg", 2.0)
            )
            return self.move_pose(
                target,
                velocity_scale=vel_scale,
                acceleration_scale=acc_scale,
                position_tolerance_mm=position_tolerance_mm,
                orientation_tolerance_deg=orientation_tolerance_deg,
            )
        raise ValueError(
            f"Unsupported position type for '{name}': {pose_type!r}. Expected 'joint' or 'cartesian'."
        )

    def move_home(self):
        return self._move_named_position("home")

    def move_to_observation_pose(self):
        self.set_octomap_mapping(False)
        return self._move_named_position("observation_pose")

    def move_to_inspection_pose(self):
        self.set_octomap_mapping(False)
        return self._move_named_position("inspection_pose")

    def get_current_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.eef_link,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:
            raise RuntimeError(
                f"TF lookup failed: {self.base_frame} <- {self.eef_link}: {error}"
            ) from error
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            rx, ry, rz = Rotation.from_quat(quat).as_euler("ZYZ", degrees=True)
        return [
            float(translation.x * 1000.0),
            float(translation.y * 1000.0),
            float(translation.z * 1000.0),
            float(rx),
            float(ry),
            float(rz),
        ]

    def _make_pose_constraints(
        self,
        target_pose,
        position_tolerance_mm=2.0,
        orientation_tolerance_deg=2.0,
    ):
        ros_pose = self._pose6_to_ros_pose(target_pose)
        constraints = Constraints()
        constraints.name = "kit_robot_pose_goal"

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.eef_link
        position_constraint.weight = 1.0

        tolerance_m = float(position_tolerance_mm) / 1000.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0 * tolerance_m] * 3

        box_pose = Pose()
        box_pose.position.x = ros_pose.position.x
        box_pose.position.y = ros_pose.position.y
        box_pose.position.z = ros_pose.position.z
        box_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(box_pose)
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.eef_link
        orientation_constraint.orientation = ros_pose.orientation

        tolerance_rad = math.radians(float(orientation_tolerance_deg))
        orientation_constraint.absolute_x_axis_tolerance = tolerance_rad
        orientation_constraint.absolute_y_axis_tolerance = tolerance_rad
        orientation_constraint.absolute_z_axis_tolerance = tolerance_rad
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def move_pose(
        self,
        target_pose,
        velocity_scale=None,
        acceleration_scale=None,
        position_tolerance_mm=2.0,
        orientation_tolerance_deg=2.0,
    ):
        """Plan/execute a free-space MoveIt pose goal for ``eef_link``.

        Unlike move_linear(), this does not force a Cartesian straight line.
        OMPL/MoveGroup is free to choose a collision-aware joint-space path
        that reaches the requested Cartesian pose.

        External pose convention:
            [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        where orientation is ZYZ Euler in ``base_frame``.
        """
        self.set_octomap_mapping(False)
        if position_tolerance_mm <= 0.0:
            raise ValueError("position_tolerance_mm must be positive")
        if orientation_tolerance_deg <= 0.0:
            raise ValueError("orientation_tolerance_deg must be positive")
        vel_scale = self._clamp_scale(
            self.default_joint_velocity_scale
            if velocity_scale is None
            else velocity_scale
        )
        acc_scale = self._clamp_scale(
            self.default_joint_acceleration_scale
            if acceleration_scale is None
            else acceleration_scale
        )
        constraints = self._make_pose_constraints(
            target_pose,
            position_tolerance_mm,
            orientation_tolerance_deg,
        )
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True
        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.logger.info(
            f"MoveIt pose goal: {list(target_pose)}, vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f}"
        )
        send_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future, self.server_timeout, "sending MoveGroup pose goal"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup pose goal was rejected")
        result_future = goal_handle.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future, self.motion_timeout, "MoveGroup pose execution"
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise
        result = wrapped_result.result
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "MoveGroup pose plan/execute failed: " + self._error_name(code)
            )
        return 0

    def _execute_trajectory(self, trajectory, description):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.controller_names = []

        handle = self._wait_future(
            self._execute_client.send_goal_async(goal),
            self.server_timeout,
            f"sending {description}",
        )
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{description}: ExecuteTrajectory rejected")

        try:
            wrapped = self._wait_future(
                handle.get_result_async(),
                self.motion_timeout,
                description,
            )
        except Exception:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            raise

        code = int(wrapped.result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{description}: " + self._error_name(code))

    def move_linear(self, target_pose, vel=100, acc=200, avoid_collisions=True):
        self.set_octomap_mapping(False)
        if vel <= 0 or acc <= 0:
            raise ValueError("vel and acc must be positive")
        ros_pose = self._pose6_to_ros_pose(target_pose)
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self._moveit_node.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.eef_link
        request.waypoints = [ros_pose]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = bool(avoid_collisions)
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = self._clamp_scale(
            float(acc) / self.cartesian_acc_reference_mm_s2
        )
        request.cartesian_speed_limited_link = self.eef_link
        request.max_cartesian_speed = float(vel) / 1000.0
        self.logger.info(
            f"MoveIt Cartesian goal: {list(target_pose)}, speed={float(vel):.1f} mm/s"
        )
        future = self._cartesian_client.call_async(request)
        response = self._wait_future(future, self.motion_timeout, "GetCartesianPath")
        if response is None:
            raise RuntimeError("GetCartesianPath returned no response")
        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("Cartesian planning failed: " + self._error_name(code))
        if response.fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                f"Cartesian path incomplete: fraction={response.fraction:.3f} < {self.cartesian_min_fraction:.3f}"
            )
        points = response.solution.joint_trajectory.points
        if not points:
            self.logger.info("Cartesian target already satisfied (empty trajectory).")
            return 0
        self._execute_trajectory(
            response.solution,
            "Cartesian trajectory execution",
        )
        return 0

    def _cup_call_perception_once(self):
        if not self._cup_perception_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            raise RuntimeError(
                f"Cup perception service unavailable: {self.cup_grasp_service}"
            )
        old_mtime_ns = (
            os.stat(self.cup_grasp_snapshot).st_mtime_ns
            if os.path.isfile(self.cup_grasp_snapshot)
            else None
        )
        future = self._cup_perception_client.call_async(Trigger.Request())
        response = self._wait_future(
            future,
            self.cup_grasp_perception_timeout_sec,
            "Cup FoundationPose/GraspGenX perception",
        )
        if response is None:
            raise RuntimeError("Cup perception returned no response")
        if not response.success:
            message = str(response.message)
            retryable_markers = (
                "TARGET_NOT_DETECTED",
                "not detected",
                "No GraspGenX candidate satisfies",
                "zero grasp",
                "no grasp",
            )
            if any((marker.lower() in message.lower() for marker in retryable_markers)):
                self.logger.warning(
                    f"Cup perception produced no usable grasp: {message}"
                )
                return None
            raise RuntimeError("Cup perception failed: " + message)
        deadline = time.monotonic() + self.cup_grasp_snapshot_timeout_sec
        while time.monotonic() < deadline:
            if os.path.isfile(self.cup_grasp_snapshot):
                new_mtime_ns = os.stat(self.cup_grasp_snapshot).st_mtime_ns
                if old_mtime_ns is None or new_mtime_ns != old_mtime_ns:
                    self.logger.info(
                        f"Fresh live GraspGenX snapshot: {self.cup_grasp_snapshot}"
                    )
                    return response
            time.sleep(0.02)
        raise RuntimeError(
            f"Cup perception succeeded but no fresh candidate snapshot appeared: {self.cup_grasp_snapshot}"
        )

    def _cup_load_snapshot(self):
        with np.load(self.cup_grasp_snapshot, allow_pickle=False) as data:
            if "grasps" not in data or "confidences" not in data:
                raise ValueError(
                    "Live GraspGenX snapshot must contain 'grasps' and 'confidences'"
                )
            grasps = np.asarray(data["grasps"], dtype=np.float64).reshape(-1, 4, 4)
            scores = np.asarray(data["confidences"], dtype=np.float64).reshape(-1)
        if len(grasps) != len(scores):
            raise ValueError(
                f"Live GraspGenX snapshot length mismatch: grasps={len(grasps)}, scores={len(scores)}"
            )
        if len(grasps) == 0:
            raise RuntimeError("Live GraspGenX returned zero candidates")
        if not np.all(np.isfinite(grasps)):
            raise ValueError("Live GraspGenX grasps contain non-finite values")
        if not np.all(np.isfinite(scores)):
            raise ValueError("Live GraspGenX scores contain non-finite values")
        return grasps, scores

    def _cup_filter_top_candidates(self, T_base_camera, grasps, scores):
        R_base_camera = np.asarray(T_base_camera, dtype=np.float64).reshape(4, 4)[
            :3, :3
        ]
        approach_camera = grasps[:, :3, 2]
        approach_base = (R_base_camera @ approach_camera.T).T
        norms = np.linalg.norm(approach_base, axis=1)
        finite = (
            np.isfinite(approach_base).all(axis=1)
            & np.isfinite(scores)
            & (norms > 1e-09)
        )
        unit = np.zeros_like(approach_base, dtype=np.float64)
        unit[finite] = approach_base[finite] / norms[finite, None]
        cos_angle = np.clip(-unit[:, 2], -1.0, 1.0)
        tilt_deg = np.full(len(grasps), np.inf, dtype=np.float64)
        tilt_deg[finite] = np.degrees(np.arccos(cos_angle[finite]))
        valid = np.flatnonzero(finite & (tilt_deg <= self.cup_grasp_max_tilt_deg))
        if len(valid) == 0:
            finite_tilts = tilt_deg[np.isfinite(tilt_deg)]
            closest = float(np.min(finite_tilts)) if len(finite_tilts) else float("inf")
            self.logger.warning(
                f"No cup grasp inside base -Z cone: limit={self.cup_grasp_max_tilt_deg:.1f}deg, closest={closest:.2f}deg"
            )
            return ([], tilt_deg, unit)
        ordered = valid[np.argsort(-scores[valid])]
        top = ordered[: min(self.cup_grasp_top_k, len(ordered))]
        self.logger.info(
            f"[CUP GRASP FILTER] raw={len(grasps)}, valid={len(valid)}, top_k={len(top)}, tilt_limit={self.cup_grasp_max_tilt_deg:.1f}deg"
        )
        return ([int(index) for index in top], tilt_deg, unit)

    def _cup_prepare_candidate(self, T_base_camera, T_camera_grasp):
        """Convert a camera-frame grasp into grasp and pregrasp EEF poses."""
        T_base_camera = np.asarray(T_base_camera, dtype=np.float64).reshape(4, 4)
        T_camera_grasp = np.asarray(T_camera_grasp, dtype=np.float64).reshape(4, 4)
        T_tool_eef = self._lookup_transform_matrix(
            self.cup_tool_frame,
            self.eef_link,
        )
        grasp_offset_m = self.cup_grasp_offset_mm / 1000.0
        pregrasp_distance_m = self.cup_grasp_pregrasp_distance_mm / 1000.0
        T_camera_grasp_exec = T_camera_grasp @ self._translation_matrix_m(
            z=-grasp_offset_m
        )
        T_base_grasp_exec = T_base_camera @ T_camera_grasp_exec
        T_base_tool = T_base_grasp_exec @ self.cup_grasp_to_tool
        T_base_eef = T_base_tool @ T_tool_eef
        T_camera_pregrasp = T_camera_grasp_exec @ self._translation_matrix_m(
            z=-pregrasp_distance_m
        )
        T_base_tool_pre = T_base_camera @ T_camera_pregrasp @ self.cup_grasp_to_tool
        T_base_eef_pre = T_base_tool_pre @ T_tool_eef
        return {
            "grasp_pose6": self._matrix_m_to_pose6(T_base_eef),
            "pregrasp_pose6": self._matrix_m_to_pose6(T_base_eef_pre),
        }

    def _cup_plan_pregrasp_only(self, target_pose):
        """Current real robot state -> PREGRASP, plan_only=True."""
        self.set_octomap_mapping(False)
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self._clamp_scale(
            self.default_joint_velocity_scale
        )
        goal.request.max_acceleration_scaling_factor = self._clamp_scale(
            self.default_joint_acceleration_scale
        )
        goal.request.goal_constraints = [self._make_pose_constraints(target_pose)]
        goal.request.start_state.is_diff = True
        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        goal.planning_options.replan_delay = 0.0
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        send_future = self._move_group_client.send_goal_async(goal)
        handle = self._wait_future(
            send_future, self.server_timeout, "sending cup PREGRASP plan-only goal"
        )
        if handle is None or not handle.accepted:
            raise RuntimeError("Cup PREGRASP MoveGroup goal rejected")
        wrapped = self._wait_future(
            handle.get_result_async(), self.motion_timeout, "cup PREGRASP plan-only"
        )
        result = wrapped.result
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Cup PREGRASP planning failed: " + self._error_name(code)
            )
        trajectory = result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("Cup PREGRASP returned empty trajectory")
        return trajectory

    @staticmethod
    def _cup_state_from_trajectory_end(trajectory):
        jt = trajectory.joint_trajectory
        state = RobotState()
        state.is_diff = True
        if not jt.joint_names or not jt.points:
            return state
        last = jt.points[-1]
        state.joint_state.name = list(jt.joint_names)
        state.joint_state.position = list(last.positions)
        state.joint_state.velocity = [0.0] * len(last.positions)
        return state

    def _cup_plan_cartesian_only(
        self, start_state, target_pose, *, avoid_collisions, description
    ):
        """Synthetic start state -> target, plan only."""
        ros_pose = self._pose6_to_ros_pose(target_pose)
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self._moveit_node.get_clock().now().to_msg()
        request.start_state = start_state
        request.group_name = self.group_name
        request.link_name = self.eef_link
        request.waypoints = [ros_pose]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = bool(avoid_collisions)
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = self._clamp_scale(
            self.cup_grasp_linear_acc_mm_s2 / self.cartesian_acc_reference_mm_s2
        )
        request.cartesian_speed_limited_link = self.eef_link
        request.max_cartesian_speed = self.cup_grasp_linear_vel_mm_s / 1000.0
        response = self._wait_future(
            self._cartesian_client.call_async(request), self.motion_timeout, description
        )
        if response is None:
            raise RuntimeError(f"{description}: no response")
        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{description}: " + self._error_name(code))
        fraction = float(response.fraction)
        if fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                f"{description}: fraction={fraction:.3f} < {self.cartesian_min_fraction:.3f}"
            )
        if not response.solution.joint_trajectory.points:
            raise RuntimeError(f"{description}: empty trajectory")
        return (response.solution, fraction)

    def _cup_validate_candidate(self, pregrasp_pose, grasp_pose):
        """Validate all motion segments before any robot motion."""
        pregrasp_trajectory = self._cup_plan_pregrasp_only(pregrasp_pose)
        pregrasp_state = self._cup_state_from_trajectory_end(pregrasp_trajectory)
        approach_trajectory, approach_fraction = self._cup_plan_cartesian_only(
            pregrasp_state,
            grasp_pose,
            avoid_collisions=True,
            description="cup PREGRASP -> GRASP Cartesian plan-only",
        )
        grasp_state = self._cup_state_from_trajectory_end(approach_trajectory)
        retreat_trajectory, retreat_fraction = self._cup_plan_cartesian_only(
            grasp_state,
            pregrasp_pose,
            avoid_collisions=False,
            description="cup GRASP -> PREGRASP retreat plan-only",
        )
        return {
            "pregrasp_trajectory": pregrasp_trajectory,
            "approach_trajectory": approach_trajectory,
            "retreat_trajectory": retreat_trajectory,
            "approach_fraction": approach_fraction,
            "retreat_fraction": retreat_fraction,
        }

    def pick_cup_graspgenx_live(self, component_name="컵라면"):
        """Execute the validated live cup-ramen grasp workflow.

        1. Capture T_base_camera once while stationary at observation pose.
        2. Trigger ONE FoundationPose -> GraspGenX inference.
        3. Load the candidates returned by that inference.
        4. Filter by the configured base -Z tilt limit.
        5. Score-sort and test up to the configured Top-K.
        6. Candidate must pass:
             current -> PREGRASP free-space plan
             PREGRASP -> GRASP Cartesian fraction
             GRASP -> PREGRASP retreat fraction
        7. Execute the exact validated trajectories, without replanning.
        8. Return RG2 grip detection result.

        False means a normal grasp/observation retry may be attempted by Controller.
        Infrastructure errors raise RuntimeError and follow Controller's fatal path.
        """
        if component_name != self.cup_grasp_component:
            raise ValueError(
                f"pick_cup_graspgenx_live is reserved for {self.cup_grasp_component!r}, got {component_name!r}"
            )
        T_base_camera = self._lookup_transform_matrix(
            self.base_frame,
            self.cup_camera_frame,
        )
        self.logger.info(
            f"[CUP] Captured observation-time T_base_camera once: xyz_m={np.round(T_base_camera[:3, 3], 5).tolist()}"
        )
        if self._cup_call_perception_once() is None:
            return False
        grasps, scores = self._cup_load_snapshot()
        top_indices, tilt_deg, approach_base = self._cup_filter_top_candidates(
            T_base_camera, grasps, scores
        )
        if not top_indices:
            return False
        self.set_octomap_mapping(False)
        selected = None
        for rank, index in enumerate(top_indices, start=1):
            score = float(scores[index])
            tilt = float(tilt_deg[index])
            approach = approach_base[index]
            prepared = self._cup_prepare_candidate(T_base_camera, grasps[index])
            pre = prepared["pregrasp_pose6"]
            grasp = prepared["grasp_pose6"]
            self.logger.info(
                f"[CUP CANDIDATE {rank}/{len(top_indices)}] index={index}, score={score:.4f}, tilt={tilt:.2f}deg, grasp_offset={self.cup_grasp_offset_mm:.1f}mm, pregrasp_extra={self.cup_grasp_pregrasp_distance_mm:.1f}mm, approach_base=({approach[0]:+.3f}, {approach[1]:+.3f}, {approach[2]:+.3f}), pre={np.round(pre, 2).tolist()}, grasp={np.round(grasp, 2).tolist()}"
            )
            try:
                planned = self._cup_validate_candidate(pre, grasp)
            except Exception as error:
                self.logger.warning(
                    f"[CUP CANDIDATE {rank} REJECTED] index={index}: {error}"
                )
                continue
            selected = {
                "rank": rank,
                "index": index,
                "score": score,
                "tilt": tilt,
                "pregrasp_pose6": pre,
                "grasp_pose6": grasp,
                **planned,
            }
            self.logger.info(
                f"[CUP CANDIDATE {rank} SELECTED] index={index}, score={score:.4f}, tilt={tilt:.2f}deg, approach_fraction={planned['approach_fraction']:.3f}, retreat_fraction={planned['retreat_fraction']:.3f}"
            )
            break
        if selected is None:
            self.logger.error(
                f"[CUP] All Top-{len(top_indices)} candidates failed MoveIt validation"
            )
            return False
        params = self.grasp_params.get(component_name, self.grasp_params["_default"])
        open_width = params["width"]
        grip_force = params["force"]
        self.set_octomap_mapping(False)
        self.logger.info(
            f"[CUP EXECUTE] selected_rank=#{selected['rank']}, index={selected['index']}, score={selected['score']:.4f}, tilt={selected['tilt']:.2f}deg"
        )
        self.rg.move_gripper(open_width, force_val=grip_force)
        time.sleep(2.0)
        self._execute_trajectory(
            selected["pregrasp_trajectory"], "cup validated PREGRASP trajectory"
        )
        self._execute_trajectory(
            selected["approach_trajectory"], "cup validated GRASP approach trajectory"
        )
        self.rg.close_gripper(force_val=grip_force)
        time.sleep(5.0)
        self._execute_trajectory(
            selected["retreat_trajectory"], "cup validated GRASP retreat trajectory"
        )
        time.sleep(1.0)
        status = self.rg.get_status()
        grip_detected = bool(status[1])
        self.logger.info(f"[CUP RESULT] status={status}, grip_detected={grip_detected}")
        return grip_detected

    @staticmethod
    def _matrix_from_transform(transform):
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        T[:3, 3] = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        return T

    def _lookup_transform_matrix(self, target_frame, source_frame):
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)
        try:
            stamped = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:
            raise RuntimeError(
                f"TF lookup failed: {target_frame} <- {source_frame}: {error}"
            ) from error
        return self._matrix_from_transform(stamped.transform)

    @staticmethod
    def _matrix_m_to_pose6(T):
        T = np.asarray(T, dtype=np.float64).reshape(4, 4)
        if not np.all(np.isfinite(T)):
            raise ValueError("transform contains non-finite values")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            zyz = Rotation.from_matrix(T[:3, :3]).as_euler("ZYZ", degrees=True)
        xyz_mm = T[:3, 3] * 1000.0
        return [
            float(xyz_mm[0]),
            float(xyz_mm[1]),
            float(xyz_mm[2]),
            float(zyz[0]),
            float(zyz[1]),
            float(zyz[2]),
        ]

    @staticmethod
    def _translation_matrix_m(x=0.0, y=0.0, z=0.0):
        T = np.eye(4, dtype=np.float64)
        T[:3, 3] = [float(x), float(y), float(z)]
        return T

    def pick_component(self, component_name, target_pose, vel=80, acc=160):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")

        params = self.grasp_params.get(component_name, self.grasp_params["_default"])
        open_width = params["width"]
        grip_force = params["force"]
        approach_height = params["approach"]

        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        self.logger.info(
            f"Pick component: {component_name}, width={open_width}, force={grip_force}, approach={approach_height}"
        )
        self.set_octomap_mapping(False)

        # Five local attempts are intentional and may be repeated by Controller.
        for attempt_index in range(5):
            self.rg.move_gripper(open_width, force_val=grip_force)
            time.sleep(2.0)

            self.move_pose(pick_pose_up)
            time.sleep(0.5)
            self.move_linear(pick_pose_down, vel=vel, acc=acc)

            self.rg.close_gripper(force_val=grip_force)
            time.sleep(5.0)

            self.move_linear(
                pick_pose_up,
                vel=vel,
                acc=acc,
                avoid_collisions=False,
            )
            time.sleep(1.0)

            gripper_width = self.rg.get_width()
            if gripper_width > 13:
                self.logger.info(
                    f"Grasp succeeded ({attempt_index + 1}/5), "
                    f"width={gripper_width} mm"
                )
                return True

            self.logger.warning(
                f"Grasp failed ({attempt_index + 1}/5), " f"width={gripper_width} mm"
            )

        return False

    def place_component(self, component_name, slot_name, approach_height=350):
        if slot_name not in self.place_slots:
            raise ValueError(f"Unknown place slot: {slot_name}")

        place_pose_down = list(self.place_slots[slot_name]["pos"])
        if len(place_pose_down) != 6:
            raise ValueError(f"{slot_name} pose must be [x, y, z, rx, ry, rz]")

        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height
        linear_vel = float(self.place_config.get("linear_vel", 60.0))
        linear_acc = float(self.place_config.get("linear_acc", 120.0))

        self.set_octomap_mapping(False)
        self.logger.info(
            f"Place component: {component_name} -> {slot_name}, approach={approach_height} mm"
        )

        # Use pose planning to reach the offset and Cartesian motion only nearby.
        self.move_pose(place_pose_up)
        time.sleep(0.5)
        self.move_linear(place_pose_down, vel=linear_vel, acc=linear_acc)

        self.rg.open_gripper()
        time.sleep(2.0)

        self.move_linear(
            place_pose_up, vel=linear_vel, acc=linear_acc, avoid_collisions=False
        )
        return True

    def recover_to_safe_pose(self):
        self.logger.warning("Recovering from task failure")
        self.set_octomap_mapping(False, force=True)
        self.rg.open_gripper()
        time.sleep(2.0)
        self.logger.info("Recovery complete; current robot pose is preserved")

    def shutdown(self):
        try:
            self.set_octomap_mapping(False, force=True)
            self.rg.close()
        except Exception:
            pass
        try:
            if hasattr(self, "_executor"):
                self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            try:
                self._executor.cancel()
            except Exception:
                pass
        try:
            if hasattr(self, "_spin_thread") and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, "_moveit_node"):
                self._moveit_node.destroy_node()
        except Exception:
            pass
