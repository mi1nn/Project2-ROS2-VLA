import json
import math
import os
import threading
import time
import warnings

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
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
)
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
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
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

    # "<octomap>" already exists if move_group kept running across a restart of
    # this node; appending a name twice would leave a stale, unused row.
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

    return names, matrix


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

        grasp_params_path = os.path.join(
            get_package_share_directory("kit_robot"),
            "resource",
            "grasp_params.json",
        )
        with open(grasp_params_path, "r", encoding="utf-8") as file:
            self.grasp_params = json.load(file)

        # MoveIt / TF configuration. These defaults match the standard Doosan M0609
        # MoveIt model. If the custom RG2 SRDF uses another tip, change eef_link only.
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

        # Fixed keepout box. Bounds are configured in centimeters and converted
        # to meters only when messages are sent to MoveIt/RViz. Other Motion pose
        # APIs remain millimeter-based for compatibility with the existing project.
        self.keepout_config = self.moveit_config.get("default_keepout_box", {})
        self.keepout_enabled = bool(self.keepout_config.get("enabled", True))
        self.keepout_object_id = str(
            self.keepout_config.get("id", "base_keepout_box")
        )
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

        # Octomap gate. cloud_out must match point_cloud_topic in the MoveIt
        # config's sensors_3d.yaml, otherwise the octomap stays empty silently.
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
        # Links allowed to collide with octomap voxels: the gripper and what is
        # bolted to it. The target object and the table are voxels too, so
        # without this no grasp is ever plannable. Upper arm links stay out.
        self.octomap_allowed_links = [
            str(name)
            for name in self.octomap_config.get(
                "allowed_collision_links",
                ["link_6", "tool0", "rg2_base_link"],
            )
        ]

        self.server_timeout = float(
            self.moveit_config.get("server_ready_timeout_sec", 15.0)
        )
        self.motion_timeout = float(
            self.moveit_config.get("motion_timeout_sec", 60.0)
        )
        self.tf_timeout = float(self.moveit_config.get("tf_timeout_sec", 2.0))
        self.planning_time = float(
            self.moveit_config.get("planning_time_sec", 5.0)
        )
        self.planning_attempts = int(
            self.moveit_config.get("planning_attempts", 5)
        )
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

        # A dedicated node/executor lets Motion synchronously wait for MoveIt actions
        # without blocking Controller's state-machine node or nesting rclpy.spin calls.
        self._moveit_node = rclpy.create_node("kit_moveit_interface")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._moveit_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="kit_moveit_executor",
            daemon=True,
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
        self._apply_planning_scene_client = self._moveit_node.create_client(
            ApplyPlanningScene, self.apply_planning_scene_service_name
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._keepout_marker_pub = self._moveit_node.create_publisher(
            Marker, self.keepout_marker_topic, marker_qos
        )

        # Open by default so a run that never reaches the observation pose (or a
        # bringup without Controller) still shows a map in RViz.
        self._octomap_mapping = self.octomap_enabled
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
            # ponytail: Python relay of a 848x480 cloud. Only open while the arm
            # is parked, and the updater throttles to max_update_rate anyway. If
            # CPU matters, point sensors_3d.yaml at the camera topic and gate the
            # driver instead.
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
            "MoveIt2 Motion initialized: "
            f"group={self.group_name}, base={self.base_frame}, eef={self.eef_link}"
        )

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------

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
        # Needed by both the keepout box and the octomap ACM merge.
        needs_planning_scene = self.keepout_enabled or self.octomap_enabled
        if needs_planning_scene and not self._apply_planning_scene_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            raise RuntimeError(
                "ApplyPlanningScene service unavailable: "
                f"{self.apply_planning_scene_service_name}"
            )

        # Missing octomap services mean move_group has no occupancy map monitor.
        # Without GetPlanningScene the ACM cannot be merged, and an un-excused
        # octomap makes every grasp unplannable — so drop the map entirely
        # instead of dying or bricking the run.
        for client, name in (
            (self._clear_octomap_client, self.octomap_clear_service),
            (self._get_planning_scene_client, self.octomap_get_scene_service),
        ):
            if not self.octomap_enabled:
                break
            if client.wait_for_service(timeout_sec=self.server_timeout):
                continue
            self.logger.warning(
                f"Octomap service unavailable: {name} — octomap disabled. "
                "Obstacle avoidance falls back to the keepout box only."
            )
            self.octomap_enabled = False
            self._octomap_mapping = False

    def _wait_future(self, future, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{description} timed out after {timeout_sec:.1f}s")
            time.sleep(0.01)

        if not rclpy.ok() and not future.done():
            raise RuntimeError(f"ROS shutdown while waiting for {description}")
        if future.cancelled():
            raise RuntimeError(f"{description} was cancelled")

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{description} failed: {exception}") from exception
        return future.result()

    @staticmethod
    def _error_name(code):
        # Keep the numeric value because generated Python messages do not expose
        # a guaranteed enum-to-string helper across MoveIt releases.
        return f"MoveItErrorCodes({code})"

    @staticmethod
    def _pose6_to_ros_pose(pose6):
        pose = list(pose6)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")
        if not all(math.isfinite(float(v)) for v in pose):
            raise ValueError("target_pose contains non-finite values")

        msg = Pose()
        msg.position.x = float(pose[0]) / 1000.0
        msg.position.y = float(pose[1]) / 1000.0
        msg.position.z = float(pose[2]) / 1000.0

        quat = Rotation.from_euler(
            "ZYZ", [float(pose[3]), float(pose[4]), float(pose[5])], degrees=True
        ).as_quat()  # scipy order: x, y, z, w
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
        return self._clamp_scale(vel_scale), self._clamp_scale(acc_scale)

    @staticmethod
    def _validate_keepout_bounds(min_cm, max_cm):
        if len(min_cm) != 3 or len(max_cm) != 3:
            raise ValueError(
                "default_keepout_box min_cm/max_cm must contain [x, y, z]"
            )

        min_values = [float(v) for v in min_cm]
        max_values = [float(v) for v in max_cm]

        if not all(math.isfinite(v) for v in min_values + max_values):
            raise ValueError("default_keepout_box contains non-finite values")

        for axis, low, high in zip("xyz", min_values, max_values):
            if high <= low:
                raise ValueError(
                    f"default_keepout_box max_{axis} must be greater than min_{axis}"
                )

        return min_values, max_values

    def _keepout_geometry_m(self):
        min_cm, max_cm = self._validate_keepout_bounds(
            self.keepout_min_cm, self.keepout_max_cm
        )

        # cm -> m
        center_m = [
            (low + high) * 0.5 / 100.0
            for low, high in zip(min_cm, max_cm)
        ]
        size_m = [
            (high - low) / 100.0
            for low, high in zip(min_cm, max_cm)
        ]
        return min_cm, max_cm, center_m, size_m

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

        # Semi-transparent red keepout visualization in RViz.
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.30

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
            future,
            self.server_timeout,
            "ApplyPlanningScene(default_keepout_box)",
        )

        if response is None or not response.success:
            raise RuntimeError("Failed to add default keepout box")

        self._publish_keepout_marker(center_m, size_m)

        center_cm = [value * 100.0 for value in center_m]
        size_cm = [value * 100.0 for value in size_m]
        self.logger.info(
            "Default keepout box added: "
            f"id={self.keepout_object_id}, "
            f"frame={self.base_frame}, "
            f"min_cm={min_cm}, max_cm={max_cm}, "
            f"center_cm={[round(v, 3) for v in center_cm]}, "
            f"size_cm={[round(v, 3) for v in size_cm]}, "
            f"marker_topic={self.keepout_marker_topic}"
        )
        return True

    # ------------------------------------------------------------------
    # Octomap (MoveIt occupancy map monitor)
    # ------------------------------------------------------------------

    def _octomap_cloud_callback(self, msg):
        if self._octomap_mapping:
            self._octomap_cloud_pub.publish(msg)

    def set_octomap_mapping(self, enabled):
        """Open/close the point-cloud gate that feeds MoveIt's octomap.

        Only open it while the arm is parked. The camera is eye-in-hand, so a
        cloud captured mid-motion is registered with the wrong TF and smears
        voxels across the workspace.
        """
        if not self.octomap_enabled:
            return False

        enabled = bool(enabled)
        if enabled != self._octomap_mapping:
            self.logger.info(
                f"Octomap mapping {'ON' if enabled else 'OFF'} "
                f"({self.octomap_cloud_in} -> {self.octomap_cloud_out})"
            )
        self._octomap_mapping = enabled
        return enabled

    def clear_octomap(self):
        """Drop every voxel MoveIt has collected.

        Voxels are never un-occupied by a new cloud that simply no longer sees
        them, so a picked-up object would haunt the map forever. Clear right
        before re-opening the gate at an observation/inspection pose; never
        during a pick, or the map the arm is avoiding disappears.
        """
        if not self.octomap_enabled:
            return False

        future = self._clear_octomap_client.call_async(Empty.Request())
        try:
            self._wait_future(future, self.server_timeout, "ClearOctomap")
        except Exception as error:
            self.logger.warning(f"ClearOctomap failed: {error}")
            return False

        self.logger.info("Octomap cleared")
        return True

    def _allow_octomap_collisions(self):
        """Excuse the gripper links from octomap collisions, once, at startup.

        MoveIt keeps one special ACM entry named "<octomap>" covering every
        voxel. Allowing it against the gripper links lets the fingers reach
        into the target object's voxels while the upper arm still avoids
        everything the camera mapped.

        The ACM has to be read first: a PlanningScene diff carrying a non-empty
        ACM *replaces* the whole matrix, which would wipe the SRDF
        self-collision pairs.
        """
        if not self.octomap_enabled or not self.octomap_allowed_links:
            return False

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get_planning_scene_client.call_async(request)
        response = self._wait_future(
            future, self.server_timeout, "GetPlanningScene(ACM)"
        )

        acm = response.scene.allowed_collision_matrix
        if not acm.entry_names:
            raise RuntimeError(
                "Planning scene returned an empty ACM; "
                "refusing to overwrite self-collision pairs"
            )

        unknown = [
            name
            for name in self.octomap_allowed_links
            if name not in acm.entry_names
        ]
        if unknown:
            # A typo here fails silently otherwise: the entry is created, matches
            # no link, and every grasp keeps colliding with the object voxels.
            self.logger.warning(
                f"octomap.allowed_collision_links not in the robot ACM: {unknown}"
            )

        names, matrix = merge_octomap_acm(
            list(acm.entry_names),
            [list(entry.enabled) for entry in acm.entry_values],
            self.octomap_allowed_links,
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
            "Octomap collisions allowed for "
            f"{len(self.octomap_allowed_links)} links: "
            f"{self.octomap_allowed_links}"
        )
        return True

    # ------------------------------------------------------------------
    # MoveIt joint planning (replacement for movej)
    # ------------------------------------------------------------------

    def move_joint(self, joint_deg, velocity_scale=None, acceleration_scale=None):
        # eye-in-hand: a cloud captured while moving lands on the wrong TF and
        # smears voxels. Stop accumulating; the existing map stays and is avoided.
        self.set_octomap_mapping(False)

        values = list(joint_deg)
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"joint target must have {len(self.joint_names)} values, got {len(values)}"
            )
        if not all(math.isfinite(float(v)) for v in values):
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
        goal.request.start_state.is_diff = True  # current MoveIt planning-scene state
        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id

        # plan_only=False makes move_group both plan and execute through its
        # configured ros2_control trajectory controller.
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1
        # Preserve the currently monitored planning scene instead of replacing it.
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.logger.info(
            f"MoveIt joint goal: {values}, vel_scale={vel_scale:.3f}, "
            f"acc_scale={acc_scale:.3f}"
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
          - cartesian/pose: [x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]
            where A/B/C use the Doosan-compatible intrinsic ZYZ Euler convention.
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

            self.logger.info(
                f"Move named position '{name}' as joint target: {target}"
            )
            return self.move_joint(
                target,
                velocity_scale=vel_scale,
                acceleration_scale=acc_scale,
            )

        if pose_type in ("cartesian", "pose"):
            if len(target) != 6:
                raise ValueError(
                    f"{name}.pos must be "
                    "[x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]"
                )

            position_tolerance_mm = float(
                config.get("position_tolerance_mm", 2.0)
            )
            orientation_tolerance_deg = float(
                config.get("orientation_tolerance_deg", 2.0)
            )

            self.logger.info(
                f"Move named position '{name}' as Cartesian target: {target}, "
                f"position_tolerance={position_tolerance_mm:.2f} mm, "
                f"orientation_tolerance={orientation_tolerance_deg:.2f} deg"
            )

            return self.move_pose(
                target,
                velocity_scale=vel_scale,
                acceleration_scale=acc_scale,
                position_tolerance_mm=position_tolerance_mm,
                orientation_tolerance_deg=orientation_tolerance_deg,
            )

        raise ValueError(
            f"Unsupported position type for '{name}': {pose_type!r}. "
            "Expected 'joint' or 'cartesian'."
        )

    def move_home(self):
        return self._move_named_position("home")

    def move_to_observation_pose(self):
        # Map refresh point: stale voxels out (the previous component is gone from
        # the table by now), then let the cloud through while Controller waits out
        # observation_settle_sec. The snapshot taken here is what the arm avoids
        # for the rest of the cycle.
        result = self._move_named_position("observation_pose")
        self.clear_octomap()
        self.set_octomap_mapping(True)
        return result

    def move_to_inspection_pose(self):
        # Second refresh point: the tray is in frame here, so the map picks up the
        # already-placed components as obstacles.
        result = self._move_named_position("inspection_pose")
        self.clear_octomap()
        self.set_octomap_mapping(True)
        return result

    # ------------------------------------------------------------------
    # Current TCP pose: keep old DSR posx-compatible mm + ZYZ-degree format
    # ------------------------------------------------------------------

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

        # ZYZ has a mathematical singularity at beta=0/pi. scipy can still return
        # an equivalent Euler triplet; reconstruction of the rotation remains valid.
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

    # ------------------------------------------------------------------
    # MoveIt pose-goal planning
    # ------------------------------------------------------------------

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

        ros_pose = self._pose6_to_ros_pose(target_pose)

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
        constraints.name = "kit_robot_pose_goal"

        # Position goal: a small box around the requested XYZ.
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.eef_link
        position_constraint.weight = 1.0

        tolerance_m = float(position_tolerance_mm) / 1000.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            2.0 * tolerance_m,
            2.0 * tolerance_m,
            2.0 * tolerance_m,
        ]

        box_pose = Pose()
        box_pose.position.x = ros_pose.position.x
        box_pose.position.y = ros_pose.position.y
        box_pose.position.z = ros_pose.position.z
        box_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(box_pose)
        constraints.position_constraints.append(position_constraint)

        # Orientation goal: keep the requested quaternion within a small tolerance.
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.eef_link
        orientation_constraint.orientation = ros_pose.orientation

        orientation_tolerance_rad = math.radians(
            float(orientation_tolerance_deg)
        )
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

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
            f"MoveIt pose goal: {list(target_pose)}, "
            f"vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f}"
        )

        send_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future,
            self.server_timeout,
            "sending MoveGroup pose goal",
        )

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup pose goal was rejected")

        result_future = goal_handle.get_result_async()

        try:
            wrapped_result = self._wait_future(
                result_future,
                self.motion_timeout,
                "MoveGroup pose execution",
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
                "MoveGroup pose plan/execute failed: "
                + self._error_name(code)
            )

        return 0

    # ------------------------------------------------------------------
    # Cartesian straight-line planning (replacement for movel)
    # ------------------------------------------------------------------

    def move_linear(self, target_pose, vel=100, acc=200, avoid_collisions=True):
        self.set_octomap_mapping(False)

        if vel <= 0 or acc <= 0:
            raise ValueError("vel and acc must be positive")

        ros_pose = self._pose6_to_ros_pose(target_pose)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self._moveit_node.get_clock().now().to_msg()
        request.start_state.is_diff = True  # current planning-scene state
        request.group_name = self.group_name
        request.link_name = self.eef_link
        request.waypoints = [ros_pose]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = bool(avoid_collisions)

        # GetCartesianPath in MoveIt Jazzy can directly limit Cartesian speed.
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = self._clamp_scale(
            float(acc) / self.cartesian_acc_reference_mm_s2
        )
        request.cartesian_speed_limited_link = self.eef_link
        request.max_cartesian_speed = float(vel) / 1000.0  # mm/s -> m/s

        self.logger.info(
            f"MoveIt Cartesian goal: {list(target_pose)}, "
            f"speed={float(vel):.1f} mm/s"
        )

        future = self._cartesian_client.call_async(request)
        response = self._wait_future(
            future, self.motion_timeout, "GetCartesianPath"
        )
        if response is None:
            raise RuntimeError("GetCartesianPath returned no response")

        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Cartesian planning failed: " + self._error_name(code)
            )

        if response.fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                "Cartesian path incomplete: "
                f"fraction={response.fraction:.3f} < {self.cartesian_min_fraction:.3f}"
            )

        # If the requested pose is effectively the current pose MoveIt may return
        # a valid empty trajectory. Treat that as success rather than sending it.
        points = response.solution.joint_trajectory.points
        if not points:
            self.logger.info("Cartesian target already satisfied (empty trajectory).")
            return 0

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal.controller_names = []  # let MoveIt choose configured controller(s)

        send_future = self._execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future, self.server_timeout, "sending ExecuteTrajectory goal"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ExecuteTrajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future, self.motion_timeout, "Cartesian trajectory execution"
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise

        code = int(wrapped_result.result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "ExecuteTrajectory failed: " + self._error_name(code)
            )
        return 0

    # ------------------------------------------------------------------
    # High-level pick/place API kept identical to old Motion
    # ------------------------------------------------------------------

    def pick_component(self, component_name, target_pose, vel=80, acc=160):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")

        params = self.grasp_params.get(
            component_name, self.grasp_params["_default"]
        )
        open_width = params["width"]
        grip_force = params["force"]
        approach_height = params["approach"]

        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        self.logger.info(
            f"Pick component: {component_name}, width={open_width}, "
            f"force={grip_force}, approach={approach_height}"
        )

        for attempt_index in range(5):
            self.rg.move_gripper(open_width, force_val=grip_force)
            time.sleep(2.0)

            self.move_pose(pick_pose_up)
            time.sleep(0.5)
            self.move_linear(pick_pose_down, vel=vel, acc=acc)

            self.rg.close_gripper(force_val=grip_force)
            time.sleep(5.0)

            self.move_linear(pick_pose_up, vel=vel, acc=acc, avoid_collisions=False)
            time.sleep(1.0)

            gripper_status = self.rg.get_status()
            grip_detected = bool(gripper_status[1])

            if grip_detected:
                self.logger.info("Successfully gripped object")
                return True
            else:
                print(f"{attempt_index + 1} try, Failed to grip object")

            self.logger.warning(
                f"Grasp check failed ({attempt_index + 1}/5)"
            )

        self.logger.error("Failed to grip object in all 5 attempts")
        return False

    def place_component(self, component_name, slot_name, approach_height=350):
        if slot_name not in self.place_slots:
            raise ValueError(f"Unknown place slot: {slot_name}")

        place_pose_down = list(self.place_slots[slot_name]["pos"])
        if len(place_pose_down) != 6:
            raise ValueError(
                f"{slot_name} pose must be [x, y, z, rx, ry, rz]"
            )

        # PLACE 목표 위치보다 Z 방향으로 approach_height만큼 높은 안전 위치.
        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height

        # ---------------------------------------------------------
        # PLACE 좌표 확인
        # ---------------------------------------------------------
        current_pose = self.get_current_pose()

        self.logger.info(
            f"[PLACE] component={component_name}, slot={slot_name}"
        )

        self.logger.info(
            "[PLACE] current_pose = "
            f"[x={current_pose[0]:.2f}, "
            f"y={current_pose[1]:.2f}, "
            f"z={current_pose[2]:.2f}, "
            f"rx={current_pose[3]:.2f}, "
            f"ry={current_pose[4]:.2f}, "
            f"rz={current_pose[5]:.2f}]"
        )

        self.logger.info(
            "[PLACE] place_pose_up = "
            f"[x={place_pose_up[0]:.2f}, "
            f"y={place_pose_up[1]:.2f}, "
            f"z={place_pose_up[2]:.2f}, "
            f"rx={place_pose_up[3]:.2f}, "
            f"ry={place_pose_up[4]:.2f}, "
            f"rz={place_pose_up[5]:.2f}]"
        )

        self.logger.info(
            "[PLACE] place_pose_down = "
            f"[x={place_pose_down[0]:.2f}, "
            f"y={place_pose_down[1]:.2f}, "
            f"z={place_pose_down[2]:.2f}, "
            f"rx={place_pose_down[3]:.2f}, "
            f"ry={place_pose_down[4]:.2f}, "
            f"rz={place_pose_down[5]:.2f}]"
        )

        self.logger.info(
            f"[PLACE] approach_height={approach_height} mm"
        )

        # ---------------------------------------------------------
        # 1. PLACE 상공 안전 위치로 이동
        #    Cartesian 직선 경로를 강제하지 않고 MoveIt이
        #    collision-aware joint-space 경로를 계획한다.
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_pose -> place_pose_up: {place_pose_up}"
        )

        self.move_pose(place_pose_up)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after move_pose(up) = {current_pose}"
        )

        time.sleep(0.5)

        # ---------------------------------------------------------
        # 2. 최종 PLACE 위치로 이동
        #    기존 move_linear() 대신 move_pose() 사용.
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_pose -> place_pose_down: {place_pose_down}"
        )

        self.move_pose(place_pose_down)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after move_pose(down) = {current_pose}"
        )

        # ---------------------------------------------------------
        # 3. 물체 놓기
        # ---------------------------------------------------------
        self.logger.info("[PLACE] Opening gripper")
        self.rg.open_gripper()
        time.sleep(2.0)

        # ---------------------------------------------------------
        # 4. 다시 PLACE 상공 안전 위치로 이동
        #    기존 move_linear() 대신 move_pose() 사용.
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] retreat move_pose -> place_pose_up: {place_pose_up}"
        )

        self.move_pose(place_pose_up)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after retreat = {current_pose}"
        )

        self.logger.info(
            f"Complete place: {component_name} -> {slot_name}"
        )

        return True

    def recover_to_safe_pose(self):
        self.logger.warning("Recovering from task failure")
        self.rg.open_gripper()
        time.sleep(2.0)
        self.logger.info("Recovery complete; current robot pose is preserved")

    def shutdown(self):
        """Stop the private MoveIt executor/node before rclpy.shutdown()."""
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