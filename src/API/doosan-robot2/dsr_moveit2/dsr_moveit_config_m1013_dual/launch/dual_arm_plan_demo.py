#!/usr/bin/env python3
#
# Dual Arm Planning Demo
# Plans and executes coordinated dual arm motion using MoveIt2
# Both arms are planned simultaneously to avoid inter-arm collisions
#
# Usage:
#   1. First launch the dual arm system:
#      ros2 launch dsr_moveit_config_m1013_dual start_dual.launch.py
#
#   2. Then run this demo:
#      ros2 run dsr_moveit_config_m1013_dual dual_arm_plan_demo
#      (or: python3 dual_arm_plan_demo.py)
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    RobotState,
)
from sensor_msgs.msg import JointState
import math
import sys


class DualArmPlanDemo(Node):
    def __init__(self):
        super().__init__("dual_arm_plan_demo")

        self.namespace = self.declare_parameter("ns", "dsr01").value

        # MoveGroup action client
        self.move_group_client = ActionClient(
            self, MoveGroup, f"/{self.namespace}/move_action"
        )

        # Current joint state subscriber
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            f"/{self.namespace}/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected!")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def plan_and_execute(self, group_name, joint_goals):
        """
        Plan and execute a motion for the given planning group.

        Args:
            group_name: Planning group name ("dual_arms", "left_arm", "right_arm")
            joint_goals: dict of {joint_name: target_value_in_radians}
        """
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received yet!")
            return False

        # Build goal constraints
        goal_constraints = Constraints()
        for joint_name, value in joint_goals.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        # Build motion plan request
        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.goal_constraints.append(goal_constraints)

        # Set start state to current
        request.start_state = RobotState()
        request.start_state.joint_state = self.current_joint_state

        # Build MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan AND execute
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(f"Planning for group '{group_name}' with {len(joint_goals)} joints...")

        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup!")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        error_code = result.error_code.val
        if error_code == 1:  # SUCCESS
            self.get_logger().info("Motion planning and execution succeeded!")
            return True
        else:
            self.get_logger().error(f"Motion planning failed with error code: {error_code}")
            return False

    def run_demo(self):
        """Run dual arm planning demo sequence."""
        # Wait for initial joint state
        self.get_logger().info("Waiting for joint states...")
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f"Received joint states: {list(self.current_joint_state.name)}")

        deg = math.pi / 180.0

        # === Demo 1: dual_arms - both arms move simultaneously ===
        self.get_logger().info("=" * 60)
        self.get_logger().info("DEMO 1: dual_arms - Coordinated dual arm motion")
        self.get_logger().info("  Both arms plan together, avoiding inter-arm collisions")
        self.get_logger().info("=" * 60)

        # Target: left arm reaches forward, right arm reaches forward
        dual_target_1 = {
            "left_joint_1":  0.0,
            "left_joint_2":  0.0,
            "left_joint_3":  90.0 * deg,
            "left_joint_4":  0.0,
            "left_joint_5":  90.0 * deg,
            "left_joint_6":  0.0,
            "right_joint_1": 0.0,
            "right_joint_2": 0.0,
            "right_joint_3": 90.0 * deg,
            "right_joint_4": 0.0,
            "right_joint_5": 90.0 * deg,
            "right_joint_6": 0.0,
        }
        self.plan_and_execute("dual_arms", dual_target_1)

        # Wait a moment
        import time
        time.sleep(2.0)

        # === Demo 2: Return to home pose ===
        self.get_logger().info("=" * 60)
        self.get_logger().info("DEMO 2: dual_arms - Return to dual_home pose")
        self.get_logger().info("=" * 60)

        dual_home = {
            "left_joint_1":  0.0,
            "left_joint_2":  -45.0 * deg,
            "left_joint_3":  90.0 * deg,
            "left_joint_4":  0.0,
            "left_joint_5":  45.0 * deg,
            "left_joint_6":  0.0,
            "right_joint_1": 0.0,
            "right_joint_2": -45.0 * deg,
            "right_joint_3": 90.0 * deg,
            "right_joint_4": 0.0,
            "right_joint_5": 45.0 * deg,
            "right_joint_6": 0.0,
        }
        self.plan_and_execute("dual_arms", dual_home)

        self.get_logger().info("Demo complete!")


def main(args=None):
    rclpy.init(args=args)
    node = DualArmPlanDemo()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
