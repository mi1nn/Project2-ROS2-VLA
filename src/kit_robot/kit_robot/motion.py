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
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .onrobot import RG


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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._moveit_node, spin_thread=False
        )

        self.rg = RG("rg2", "192.168.1.1", 502)

        self._wait_for_moveit_servers()
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

    # ------------------------------------------------------------------
    # MoveIt joint planning (replacement for movej)
    # ------------------------------------------------------------------

    def move_joint(self, joint_deg, velocity_scale=None, acceleration_scale=None):
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

    def move_home(self):
        config = self.positions["home"]
        if config["type"] != "joint":
            raise TypeError("home position must be 'joint'")
        vel_scale, acc_scale = self._joint_scales(config)
        return self.move_joint(config["pos"], vel_scale, acc_scale)

    def move_to_observation_pose(self):
        config = self.positions["observation_pose"]
        if config["type"] != "joint":
            raise TypeError("observation_pose must be 'joint'")
        vel_scale, acc_scale = self._joint_scales(config)
        return self.move_joint(config["pos"], vel_scale, acc_scale)

    def move_to_inspection_pose(self):
        config = self.positions["inspection_pose"]
        if config["type"] != "joint":
            raise TypeError("inspection_pose must be 'joint'")
        vel_scale, acc_scale = self._joint_scales(config)
        return self.move_joint(config["pos"], vel_scale, acc_scale)

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

    def move_linear(self, target_pose, vel=100, acc=200, avoid_collisions=False):
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
            self.logger.info(f"gripper_width={gripper_width}")

            gripper_width = self.rg.get_status()

            if gripper_width > 13:
                self.logger.info("Successfully gripped object")
                return True

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

        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height

        vel = self.place_config["linear_vel"]
        acc = self.place_config["linear_acc"]

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
            f"[PLACE] approach_height={approach_height} mm, "
            f"vel={vel}, acc={acc}"
        )

        # ---------------------------------------------------------
        # 1. PLACE 상공 이동
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_pose -> place_pose_up: {place_pose_up}"
        )

        self.move_pose(place_pose_up)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after move_pose = {current_pose}"
        )

        time.sleep(0.5)

        # ---------------------------------------------------------
        # 2. PLACE 위치까지 직선 하강
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_linear -> place_pose_down: {place_pose_down}"
        )

        self.move_linear(
            place_pose_down,
            vel=vel,
            acc=acc
        )

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after downward move = {current_pose}"
        )

        # ---------------------------------------------------------
        # 3. 물체 놓기
        # ---------------------------------------------------------
        self.logger.info("[PLACE] Opening gripper")
        self.rg.open_gripper()
        time.sleep(2.0)

        # ---------------------------------------------------------
        # 4. 다시 상공으로 상승
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] retreat move_linear -> place_pose_up: {place_pose_up}"
        )

        self.move_linear(
            place_pose_up,
            vel=vel,
            acc=acc
        )

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after retreat = {current_pose}"
        )

        self.logger.info(
            f"Complete place: {component_name} -> {slot_name}"
        )

        def recover_to_safe_pose(self):
            self.logger.warning("Recovering to safe pose")
        self.rg.open_gripper()
        time.sleep(2.0)
        result = self.move_home()
        if result != 0:
            raise RuntimeError(f"Failed to recover home: result={result}")
        self.logger.info("Complete recovery to safe pose")

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
