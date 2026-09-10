#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Standalone cup-ramen LIVE pick test with Top-5 grasp retry.

Pipeline
--------
RealSense/YOLO -> FoundationPose -> GraspGenX
-> raw grasp candidates snapshot
-> base-link -Z tilt filter (<= 30 deg)
-> score sort
-> MoveIt plan-only validation for TOP 5
-> first fully plannable candidate is selected
-> with --execute, execute THE EXACT trajectories that were validated

No controller.py or motion.py modification is required.

Modes
-----
default:
    perception + Top-5 MoveIt PLAN-ONLY validation only
    NO robot motion
    NO RG2 command

--execute:
    perception once
    Top-5 validation once
    execute the exact selected trajectories
    RG2 open -> PREGRASP -> GRASP -> RG2 close -> RETREAT

Important
---------
This script expects the 30-degree perception pipeline to save:
    /tmp/graspgenx_live/latest.npz

The snapshot must contain at least:
    point_cloud
    grasps
    confidences

Candidate convention:
    GraspGenX local +Z = approach direction

Filter:
    angle(approach_base, base_link -Z) <= 30 deg

Pregrasp:
    grasp-frame -Z, 70 mm
"""

import argparse
import math
import os
import sys
import threading
import time
import warnings
from pathlib import Path

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from scipy.spatial.transform import Rotation

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath


# ============================================================
# Configuration
# ============================================================

PERCEPTION_SERVICE = "/cup_pick/perception"
GRASP_POSE_TOPIC = "/grasp/best_pose"
SNAPSHOT_PATH = Path("/tmp/graspgenx_live/latest.npz")

BASE_FRAME = "base_link"
CAMERA_FRAME = "camera_color_optical_frame"
TOOL_FRAME = "tool0"

DEFAULT_COMPONENT = "컵라면"

# User-requested top-down cone.
MAX_TILT_DEG = 30.0

# Try at most five highest-confidence candidates that pass the tilt filter.
TOP_K_TO_PLAN = 5

# User changed this from 100 mm to 70 mm.
PREGRASP_DISTANCE_M = 0.070

# Validated grasp-frame -> tool0 correction.
T_GRASP_TOOL0 = np.array(
    [
        [0.0, 0.0, -1.0,  0.000],
        [0.0, 1.0,  0.0,  0.000],
        [1.0, 0.0,  0.0, -0.004],
        [0.0, 0.0,  0.0,  1.000],
    ],
    dtype=np.float64,
)

# Conservative execution speeds.
PREGRASP_VEL_SCALE = 0.15
PREGRASP_ACC_SCALE = 0.15

LINEAR_VEL_MM_S = 50.0
LINEAR_ACC_MM_S2 = 100.0

POSE_POSITION_TOLERANCE_MM = 2.0
POSE_ORIENTATION_TOLERANCE_DEG = 2.0

TF_TIMEOUT_SEC = 5.0
PERCEPTION_TIMEOUT_SEC = 180.0
FRESH_SNAPSHOT_TIMEOUT_SEC = 3.0


# ============================================================
# Matrix helpers
# ============================================================

def transform_msg_to_matrix(msg: TransformStamped) -> np.ndarray:
    q = msg.transform.rotation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z,
    ]
    return T


def pose_stamped_to_matrix(msg: PoseStamped) -> np.ndarray:
    q = msg.pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ]
    return T


def local_translation(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose6_zyz_mm(T: np.ndarray):
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    xyz_mm = T[:3, 3] * 1000.0

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        zyz = Rotation.from_matrix(T[:3, :3]).as_euler(
            "ZYZ",
            degrees=True,
        )

    return [
        float(xyz_mm[0]),
        float(xyz_mm[1]),
        float(xyz_mm[2]),
        float(zyz[0]),
        float(zyz[1]),
        float(zyz[2]),
    ]


def print_matrix(name, T):
    print(f"\n{name}")
    print("-" * len(name))
    for row in np.asarray(T).reshape(4, 4):
        print("  [" + "  ".join(f"{v: .6f}" for v in row) + "]")


def print_pose6(name, pose):
    print(
        f"{name}: "
        f"XYZ=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) mm | "
        f"ZYZ=({pose[3]:.2f}, {pose[4]:.2f}, {pose[5]:.2f}) deg"
    )


# ============================================================
# ROS node
# ============================================================

class Top5PickNode(Node):
    def __init__(self):
        super().__init__("live_grasp_pick_top5_test")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.perception_client = self.create_client(
            Trigger,
            PERCEPTION_SERVICE,
        )

        # Optional sanity log: pipeline-selected best pose.
        self._grasp_lock = threading.Lock()
        self._latest_best_pose = None

        self.create_subscription(
            PoseStamped,
            GRASP_POSE_TOPIC,
            self._grasp_cb,
            10,
        )

    def _grasp_cb(self, msg):
        with self._grasp_lock:
            self._latest_best_pose = msg

        self.get_logger().info(
            "Pipeline /grasp/best_pose: "
            f"frame={msg.header.frame_id}, "
            f"xyz=({msg.pose.position.x:.4f}, "
            f"{msg.pose.position.y:.4f}, "
            f"{msg.pose.position.z:.4f})"
        )

    def lookup_matrix(self, target_frame, source_frame):
        deadline = time.monotonic() + TF_TIMEOUT_SEC
        last_error = None

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                return transform_msg_to_matrix(tf)
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.05)

        raise RuntimeError(
            f"TF unavailable: {target_frame} <- {source_frame}. "
            f"Last error: {last_error}"
        )

    def call_perception_once(self):
        if not self.perception_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"Service not available: {PERCEPTION_SERVICE}"
            )

        old_mtime = (
            SNAPSHOT_PATH.stat().st_mtime_ns
            if SNAPSHOT_PATH.exists()
            else None
        )

        self.get_logger().info(
            "Calling perception ONCE: "
            "FoundationPose -> GraspGenX -> candidate snapshot..."
        )

        future = self.perception_client.call_async(Trigger.Request())
        deadline = time.monotonic() + PERCEPTION_TIMEOUT_SEC

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if not future.done():
            raise TimeoutError(
                f"{PERCEPTION_SERVICE} timed out."
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "Perception service returned no response."
            )

        if not response.success:
            raise RuntimeError(
                "Perception failed: " + str(response.message)
            )

        self.get_logger().info(
            "Perception succeeded: " + str(response.message)
        )

        # Require a fresh snapshot from THIS perception request.
        deadline = time.monotonic() + FRESH_SNAPSHOT_TIMEOUT_SEC

        while rclpy.ok() and time.monotonic() < deadline:
            if SNAPSHOT_PATH.exists():
                new_mtime = SNAPSHOT_PATH.stat().st_mtime_ns
                if old_mtime is None or new_mtime != old_mtime:
                    return
            rclpy.spin_once(self, timeout_sec=0.05)

        raise RuntimeError(
            f"Perception succeeded but no fresh snapshot appeared at "
            f"{SNAPSHOT_PATH}"
        )


# ============================================================
# Candidate snapshot / selection
# ============================================================

def load_snapshot():
    with np.load(SNAPSHOT_PATH, allow_pickle=False) as data:
        grasps = np.asarray(
            data["grasps"],
            dtype=np.float64,
        ).reshape(-1, 4, 4)

        scores = np.asarray(
            data["confidences"],
            dtype=np.float64,
        ).reshape(-1)

        selected_by_pipeline = (
            int(data["selected_best_index"].item())
            if "selected_best_index" in data
            else -1
        )

        pipeline_tilts = (
            np.asarray(
                data["tilt_angles_deg"],
                dtype=np.float64,
            ).reshape(-1)
            if "tilt_angles_deg" in data
            else None
        )

    if len(grasps) != len(scores):
        raise RuntimeError(
            f"Snapshot mismatch: grasps={len(grasps)}, scores={len(scores)}"
        )

    if not len(grasps):
        raise RuntimeError(
            "Snapshot contains zero grasp candidates."
        )

    return grasps, scores, selected_by_pipeline, pipeline_tilts


def compute_base_tilt_filter(
    grasps,
    scores,
    T_base_camera,
    max_tilt_deg=MAX_TILT_DEG,
):
    """
    GraspGenX local +Z is the approach vector.
    Convert camera -> base and compare against base -Z.
    """
    R_base_camera = T_base_camera[:3, :3]

    approach_camera = grasps[:, :3, 2]
    approach_base = (R_base_camera @ approach_camera.T).T

    norms = np.linalg.norm(approach_base, axis=1)

    finite = (
        np.isfinite(approach_base).all(axis=1)
        & np.isfinite(scores)
        & (norms > 1e-9)
    )

    unit = np.zeros_like(approach_base)
    unit[finite] = approach_base[finite] / norms[finite, None]

    # dot(unit, [0,0,-1]) = -unit_z
    cos_angle = np.clip(-unit[:, 2], -1.0, 1.0)

    tilt = np.full(len(grasps), np.inf, dtype=np.float64)
    tilt[finite] = np.degrees(np.arccos(cos_angle[finite]))

    valid = np.flatnonzero(
        finite & (tilt <= float(max_tilt_deg))
    )

    if len(valid) == 0:
        closest = float(np.min(tilt[np.isfinite(tilt)]))
        raise RuntimeError(
            f"No grasp satisfies base -Z tilt <= {max_tilt_deg:.1f} deg. "
            f"Closest={closest:.2f} deg."
        )

    # Score descending, then Top-K.
    ordered = valid[np.argsort(-scores[valid])]
    top = ordered[: min(TOP_K_TO_PLAN, len(ordered))]

    return top.astype(int), tilt, unit


# ============================================================
# Grasp target conversion
# ============================================================

def candidate_targets(
    node,
    T_base_camera,
    T_camera_grasp,
    eef_link,
):
    T_camera_pregrasp = (
        T_camera_grasp
        @ local_translation(z=-PREGRASP_DISTANCE_M)
    )

    T_base_tool0_grasp = (
        T_base_camera
        @ T_camera_grasp
        @ T_GRASP_TOOL0
    )

    T_base_tool0_pregrasp = (
        T_base_camera
        @ T_camera_pregrasp
        @ T_GRASP_TOOL0
    )

    if eef_link == TOOL_FRAME:
        T_eef_tool0 = np.eye(4)
    else:
        T_eef_tool0 = node.lookup_matrix(
            eef_link,
            TOOL_FRAME,
        )

    inv_T_eef_tool0 = np.linalg.inv(T_eef_tool0)

    T_base_eef_grasp = (
        T_base_tool0_grasp
        @ inv_T_eef_tool0
    )

    T_base_eef_pregrasp = (
        T_base_tool0_pregrasp
        @ inv_T_eef_tool0
    )

    return {
        "pregrasp_pose6": matrix_to_pose6_zyz_mm(
            T_base_eef_pregrasp
        ),
        "grasp_pose6": matrix_to_pose6_zyz_mm(
            T_base_eef_grasp
        ),
        "T_base_tool0_pregrasp": T_base_tool0_pregrasp,
        "T_base_tool0_grasp": T_base_tool0_grasp,
        "T_eef_tool0": T_eef_tool0,
    }


# ============================================================
# MoveIt plan-only helpers
# ============================================================

def make_pose_constraints(
    motion,
    target_pose6,
):
    ros_pose = motion._pose6_to_ros_pose(target_pose6)

    constraints = Constraints()
    constraints.name = "top5_pregrasp_goal"

    pc = PositionConstraint()
    pc.header.frame_id = motion.base_frame
    pc.link_name = motion.eef_link
    pc.weight = 1.0

    tolerance_m = POSE_POSITION_TOLERANCE_MM / 1000.0

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [
        2.0 * tolerance_m,
        2.0 * tolerance_m,
        2.0 * tolerance_m,
    ]

    box_pose = Pose()
    box_pose.position = ros_pose.position
    box_pose.orientation.w = 1.0

    pc.constraint_region.primitives.append(box)
    pc.constraint_region.primitive_poses.append(box_pose)
    constraints.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header.frame_id = motion.base_frame
    oc.link_name = motion.eef_link
    oc.orientation = ros_pose.orientation

    tol_rad = math.radians(
        POSE_ORIENTATION_TOLERANCE_DEG
    )
    oc.absolute_x_axis_tolerance = tol_rad
    oc.absolute_y_axis_tolerance = tol_rad
    oc.absolute_z_axis_tolerance = tol_rad
    oc.weight = 1.0

    constraints.orientation_constraints.append(oc)

    return constraints


def plan_pregrasp(motion, target_pose6):
    """
    Current physical robot state -> candidate PREGRASP.
    PLAN ONLY. Returns RobotTrajectory.
    """
    motion.set_octomap_mapping(False)

    goal = MoveGroup.Goal()

    goal.request.group_name = motion.group_name
    goal.request.num_planning_attempts = motion.planning_attempts
    goal.request.allowed_planning_time = motion.planning_time
    goal.request.max_velocity_scaling_factor = motion._clamp_scale(
        PREGRASP_VEL_SCALE
    )
    goal.request.max_acceleration_scaling_factor = motion._clamp_scale(
        PREGRASP_ACC_SCALE
    )
    goal.request.goal_constraints = [
        make_pose_constraints(motion, target_pose6)
    ]
    goal.request.start_state.is_diff = True

    if motion.pipeline_id:
        goal.request.pipeline_id = motion.pipeline_id

    if motion.planner_id:
        goal.request.planner_id = motion.planner_id

    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.replan_attempts = 0
    goal.planning_options.replan_delay = 0.0
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    send_future = motion._move_group_client.send_goal_async(goal)

    handle = motion._wait_future(
        send_future,
        motion.server_timeout,
        "sending Top-5 PREGRASP plan",
    )

    if handle is None or not handle.accepted:
        raise RuntimeError(
            "PREGRASP MoveGroup goal rejected"
        )

    wrapped = motion._wait_future(
        handle.get_result_async(),
        motion.motion_timeout,
        "Top-5 PREGRASP planning",
    )

    result = wrapped.result
    code = int(result.error_code.val)

    if code != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            "PREGRASP planning failed: "
            + motion._error_name(code)
        )

    trajectory = result.planned_trajectory

    if not trajectory.joint_trajectory.points:
        raise RuntimeError(
            "PREGRASP plan returned an empty trajectory."
        )

    return trajectory, float(result.planning_time)


def state_from_trajectory_end(trajectory):
    jt = trajectory.joint_trajectory

    if not jt.joint_names or not jt.points:
        state = RobotState()
        state.is_diff = True
        return state

    end = jt.points[-1]

    state = RobotState()
    state.is_diff = True
    state.joint_state.name = list(jt.joint_names)
    state.joint_state.position = list(end.positions)
    state.joint_state.velocity = [0.0] * len(end.positions)

    return state


def plan_cartesian(
    motion,
    start_state,
    target_pose6,
    *,
    avoid_collisions,
    description,
):
    """
    Plan one Cartesian segment from a synthetic RobotState.
    PLAN ONLY: GetCartesianPath returns trajectory but does not execute it.
    """
    ros_pose = motion._pose6_to_ros_pose(target_pose6)

    request = GetCartesianPath.Request()
    request.header.frame_id = motion.base_frame
    request.header.stamp = (
        motion._moveit_node.get_clock().now().to_msg()
    )
    request.start_state = start_state
    request.group_name = motion.group_name
    request.link_name = motion.eef_link
    request.waypoints = [ros_pose]

    request.max_step = motion.cartesian_max_step_m
    request.jump_threshold = motion.cartesian_jump_threshold
    request.prismatic_jump_threshold = (
        motion.cartesian_prismatic_jump_threshold
    )
    request.revolute_jump_threshold = (
        motion.cartesian_revolute_jump_threshold
    )

    request.avoid_collisions = bool(avoid_collisions)

    request.max_velocity_scaling_factor = 1.0
    request.max_acceleration_scaling_factor = motion._clamp_scale(
        LINEAR_ACC_MM_S2
        / motion.cartesian_acc_reference_mm_s2
    )
    request.cartesian_speed_limited_link = motion.eef_link
    request.max_cartesian_speed = LINEAR_VEL_MM_S / 1000.0

    response = motion._wait_future(
        motion._cartesian_client.call_async(request),
        motion.motion_timeout,
        description,
    )

    if response is None:
        raise RuntimeError(
            f"{description}: no response"
        )

    code = int(response.error_code.val)

    if code != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"{description}: "
            + motion._error_name(code)
        )

    fraction = float(response.fraction)

    if fraction < motion.cartesian_min_fraction:
        raise RuntimeError(
            f"{description}: fraction={fraction:.3f} "
            f"< {motion.cartesian_min_fraction:.3f}"
        )

    if not response.solution.joint_trajectory.points:
        raise RuntimeError(
            f"{description}: empty trajectory"
        )

    return response.solution, fraction


def validate_candidate(
    motion,
    pregrasp_pose6,
    grasp_pose6,
):
    """
    Candidate is usable only if ALL three plans succeed:
      1) current -> PREGRASP free-space
      2) PREGRASP -> GRASP Cartesian
      3) GRASP -> PREGRASP Cartesian retreat

    Returns exact trajectories to execute later.
    """
    pregrasp_traj, planning_time = plan_pregrasp(
        motion,
        pregrasp_pose6,
    )

    pregrasp_state = state_from_trajectory_end(
        pregrasp_traj
    )

    approach_traj, approach_fraction = plan_cartesian(
        motion,
        pregrasp_state,
        grasp_pose6,
        avoid_collisions=True,
        description="PREGRASP -> GRASP Cartesian planning",
    )

    grasp_state = state_from_trajectory_end(
        approach_traj
    )

    # Existing motion.py retreats with avoid_collisions=False after grasp.
    retreat_traj, retreat_fraction = plan_cartesian(
        motion,
        grasp_state,
        pregrasp_pose6,
        avoid_collisions=False,
        description="GRASP -> PREGRASP retreat planning",
    )

    return {
        "pregrasp_trajectory": pregrasp_traj,
        "approach_trajectory": approach_traj,
        "retreat_trajectory": retreat_traj,
        "pregrasp_planning_time": planning_time,
        "approach_fraction": approach_fraction,
        "retreat_fraction": retreat_fraction,
    }


def execute_exact_trajectory(
    motion,
    trajectory,
    description,
):
    """
    Execute a previously validated RobotTrajectory exactly as-is.
    NO replanning.
    """
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = trajectory
    goal.controller_names = []

    handle = motion._wait_future(
        motion._execute_client.send_goal_async(goal),
        motion.server_timeout,
        f"sending {description}",
    )

    if handle is None or not handle.accepted:
        raise RuntimeError(
            f"{description}: ExecuteTrajectory goal rejected"
        )

    wrapped = motion._wait_future(
        handle.get_result_async(),
        motion.motion_timeout,
        description,
    )

    code = int(wrapped.result.error_code.val)

    if code != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"{description}: "
            + motion._error_name(code)
        )


# ============================================================
# Main
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Top-5 GraspGenX -> MoveIt cup pick test"
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Execute the exact trajectories of the first Top-5 candidate "
            "that passes all plan-only checks."
        ),
    )

    parser.add_argument(
        "--component",
        default=DEFAULT_COMPONENT,
    )

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()

    node = Top5PickNode()
    motion = None

    try:
        if args.execute:
            node.get_logger().warning(
                "EXECUTE MODE: Top-5 candidates will be PLAN-ONLY checked first. "
                "Only the first fully valid candidate will move the real robot."
            )
        else:
            node.get_logger().warning(
                "PLAN-ONLY MODE: Top-5 candidates will be checked. "
                "Robot and RG2 will NOT move."
            )

        try:
            from kit_robot.motion import Motion
        except Exception as exc:
            raise RuntimeError(
                "Could not import kit_robot.motion.Motion. "
                "Source the workspace first."
            ) from exc

        motion = Motion(node)
        eef_link = str(motion.eef_link)

        # --------------------------------------------------------
        # 1) Freeze eye-in-hand camera transform ONCE.
        # --------------------------------------------------------
        node.get_logger().info(
            f"Capturing observation-time "
            f"T_{BASE_FRAME}_{CAMERA_FRAME} ONCE..."
        )

        T_base_camera = node.lookup_matrix(
            BASE_FRAME,
            CAMERA_FRAME,
        )

        print_matrix(
            "T_base_camera (CAPTURED ONCE)",
            T_base_camera,
        )

        # --------------------------------------------------------
        # 2) ONE perception request only.
        # --------------------------------------------------------
        node.call_perception_once()

        grasps, scores, pipeline_best, pipeline_tilts = load_snapshot()

        node.get_logger().info(
            f"Loaded exact GraspGenX snapshot: candidates={len(grasps)}"
        )

        # --------------------------------------------------------
        # 3) Base -Z <= 30 deg, then confidence Top 5.
        # --------------------------------------------------------
        top_indices, tilts, approach_base = compute_base_tilt_filter(
            grasps,
            scores,
            T_base_camera,
            MAX_TILT_DEG,
        )

        valid_count = int(
            np.count_nonzero(np.isfinite(tilts) & (tilts <= MAX_TILT_DEG))
        )

        print("\n========== TOP-5 CANDIDATES ==========")
        print(
            f"raw candidates     : {len(grasps)}"
        )
        print(
            f"tilt <= {MAX_TILT_DEG:.1f} deg : {valid_count}"
        )
        print(
            f"planning candidates: {len(top_indices)}"
        )

        for rank, idx in enumerate(top_indices, start=1):
            a = approach_base[idx]
            print(
                f"#{rank}: index={idx:3d} "
                f"score={scores[idx]:.4f} "
                f"tilt={tilts[idx]:.2f} deg "
                f"approach_base=({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})"
            )

        print("======================================\n")

        # --------------------------------------------------------
        # 4) Plan each candidate, score order, up to Top 5.
        # --------------------------------------------------------
        selected = None

        for rank, idx in enumerate(top_indices, start=1):
            node.get_logger().info(
                f"[CANDIDATE {rank}/{len(top_indices)}] "
                f"index={idx}, score={scores[idx]:.4f}, "
                f"tilt={tilts[idx]:.2f} deg"
            )

            T_camera_grasp = np.asarray(
                grasps[idx],
                dtype=np.float64,
            ).reshape(4, 4)

            targets = candidate_targets(
                node,
                T_base_camera,
                T_camera_grasp,
                eef_link,
            )

            pregrasp_pose6 = targets["pregrasp_pose6"]
            grasp_pose6 = targets["grasp_pose6"]

            print_pose6(
                f"  #{rank} PREGRASP",
                pregrasp_pose6,
            )
            print_pose6(
                f"  #{rank} GRASP   ",
                grasp_pose6,
            )

            try:
                planned = validate_candidate(
                    motion,
                    pregrasp_pose6,
                    grasp_pose6,
                )

                node.get_logger().info(
                    f"[CANDIDATE {rank} SUCCESS] "
                    f"index={idx} | "
                    f"PREGRASP plan OK | "
                    f"approach={planned['approach_fraction']:.3f} | "
                    f"retreat={planned['retreat_fraction']:.3f}"
                )

                selected = {
                    "rank": rank,
                    "index": int(idx),
                    "score": float(scores[idx]),
                    "tilt": float(tilts[idx]),
                    "approach_base": approach_base[idx].copy(),
                    "pregrasp_pose6": pregrasp_pose6,
                    "grasp_pose6": grasp_pose6,
                    **planned,
                }
                break

            except Exception as exc:
                node.get_logger().warning(
                    f"[CANDIDATE {rank} REJECTED] "
                    f"index={idx}: {type(exc).__name__}: {exc}"
                )

        if selected is None:
            raise RuntimeError(
                f"All Top-{len(top_indices)} candidates failed MoveIt validation."
            )

        print("\n========== SELECTED GRASP ==========")
        print(
            f"rank              : #{selected['rank']}"
        )
        print(
            f"candidate index   : {selected['index']}"
        )
        print(
            f"score             : {selected['score']:.4f}"
        )
        print(
            f"base -Z tilt      : {selected['tilt']:.2f} deg"
        )
        print_pose6(
            "PREGRASP",
            selected["pregrasp_pose6"],
        )
        print_pose6(
            "GRASP   ",
            selected["grasp_pose6"],
        )
        print(
            f"approach fraction : {selected['approach_fraction']:.3f}"
        )
        print(
            f"retreat fraction  : {selected['retreat_fraction']:.3f}"
        )
        print("====================================\n")

        # --------------------------------------------------------
        # Plan-only mode ends here.
        # --------------------------------------------------------
        if not args.execute:
            print(
                "TOP-5 PLAN-ONLY SUCCESS.\n"
                "No robot motion was executed.\n"
                "No RG2 command was sent.\n\n"
                "To execute this workflow with a NEW single perception request:\n"
                "  python tools/grasp_live_pick_top5.py "
                "--component 컵라면 --execute"
            )
            return 0

        # --------------------------------------------------------
        # 5) Execute EXACT already-validated trajectories.
        #    No MoveGroup replanning.
        # --------------------------------------------------------
        params = motion.grasp_params.get(
            args.component,
            motion.grasp_params["_default"],
        )

        open_width = params["width"]
        grip_force = params["force"]

        node.get_logger().info(
            f"RG2 params: component={args.component}, "
            f"open_width={open_width}, force={grip_force}"
        )

        motion.set_octomap_mapping(False)
        motion.set_octomap_exclusion_component(
            args.component
        )

        try:
            node.get_logger().info(
                "Opening RG2..."
            )
            motion.rg.move_gripper(
                open_width,
                force_val=grip_force,
            )
            time.sleep(2.0)

            node.get_logger().info(
                "Executing EXACT validated trajectory: current -> PREGRASP"
            )
            execute_exact_trajectory(
                motion,
                selected["pregrasp_trajectory"],
                "validated PREGRASP trajectory",
            )

            node.get_logger().info(
                "Executing EXACT validated trajectory: PREGRASP -> GRASP"
            )
            execute_exact_trajectory(
                motion,
                selected["approach_trajectory"],
                "validated GRASP approach trajectory",
            )

            node.get_logger().info(
                "Closing RG2..."
            )
            motion.rg.close_gripper(
                force_val=grip_force
            )
            time.sleep(5.0)

            node.get_logger().info(
                "Executing EXACT validated trajectory: GRASP -> PREGRASP"
            )
            execute_exact_trajectory(
                motion,
                selected["retreat_trajectory"],
                "validated GRASP retreat trajectory",
            )

            time.sleep(1.0)

            try:
                status = motion.rg.get_status()
                grip_detected = bool(status[1])
                node.get_logger().info(
                    f"RG2 status={status}, grip_detected={grip_detected}"
                )
            except Exception as exc:
                grip_detected = None
                node.get_logger().warning(
                    f"RG2 status read failed: {exc}"
                )

        finally:
            try:
                motion.clear_octomap_exclusion()
            except Exception:
                pass

        print("\n========== PICK RESULT ==========")
        print(
            f"selected Top-5 rank : #{selected['rank']}"
        )
        print(
            f"candidate index     : {selected['index']}"
        )
        print(
            f"score               : {selected['score']:.4f}"
        )
        print(
            f"tilt                : {selected['tilt']:.2f} deg"
        )
        print(
            "PREGRASP trajectory : EXECUTED"
        )
        print(
            "GRASP trajectory    : EXECUTED"
        )
        print(
            "RG2 close           : SENT"
        )
        print(
            "RETREAT trajectory  : EXECUTED"
        )
        print(
            f"Grip detected       : {grip_detected}"
        )
        print("=================================\n")

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130

    except Exception as exc:
        try:
            node.get_logger().error(
                f"TEST FAILED: {type(exc).__name__}: {exc}"
            )
        except BaseException:
            print(
                f"TEST FAILED: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
        return 1

    finally:
        if motion is not None:
            try:
                motion.shutdown()
            except BaseException:
                pass

        try:
            node.destroy_node()
        except BaseException:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    sys.exit(main())
