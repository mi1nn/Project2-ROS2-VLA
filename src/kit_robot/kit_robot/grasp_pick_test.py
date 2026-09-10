#!/usr/bin/env python3
"""First real-robot bridge test for FoundationPose + GraspGenX.

Recommended order:
  1) Robot is at the exact observation pose used for the FoundationPose snapshot.
  2) Run without --execute.  Check all printed targets / RViz.
  3) Only after motion.yaml validation flags + grasp_to_tool matrix are correct,
     run with --execute.
"""

import argparse
import numpy as np
import rclpy
from rclpy.node import Node

from kit_robot.motion import Motion


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--component", default="컵라면")
    p.add_argument(
        "--candidate-file",
        default="/home/rokey/FoundationPose/grasp_bridge/00/obj_0_table_safe_grasps.npz",
    )
    p.add_argument(
        "--complete-object-pc",
        default="/home/rokey/FoundationPose/grasp_bridge/00/complete_object_pc.npy",
    )
    p.add_argument("--max-candidates", type=int, default=10)
    p.add_argument(
        "--move-observation",
        action="store_true",
        help="Move to motion.yaml observation_pose before capturing T_base_camera.",
    )
    p.add_argument(
        "--execute",
        action="store_true",
        help="Actually command MoveIt + RG2. Default is conversion-only dry-run.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("graspgenx_pick_test", namespace="/dsr01")
    motion = Motion(node)

    try:
        if args.move_observation:
            node.get_logger().info("Moving to observation pose...")
            motion.move_to_observation_pose()

        # IMPORTANT: capture this once while the arm is still at the camera pose
        # used to create complete_object_pc / T_camera_grasp.
        T_base_camera = motion.get_base_camera_matrix()
        print("\nT_base_camera =")
        print(np.array2string(T_base_camera, precision=6, suppress_small=True))

        motion.pick_graspgenx_candidates(
            component_name=args.component,
            T_base_camera=T_base_camera,
            candidate_file=args.candidate_file,
            complete_object_pc=args.complete_object_pc,
            max_candidates=args.max_candidates,
            dry_run=not args.execute,
        )
    finally:
        try:
            motion.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
