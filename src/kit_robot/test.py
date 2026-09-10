import rclpy
from rclpy.node import Node

from kit_robot.motion import Motion


INSPECTION_JOINT = [
    -190.430,
    6.440,
    55.580,
    0.33,
    117.41,
    -281.64,
]


def main(args=None):
    rclpy.init(args=args)

    node = Node("inspection_pose_test")
    motion = None

    try:
        motion = Motion(node)

        node.get_logger().info(
            f"Move to inspection pose: {INSPECTION_JOINT}"
        )

        motion.move_joint(
            INSPECTION_JOINT,
            vel_scale=0.15,
            acc_scale=0.15,
        )

        node.get_logger().info(
            "Inspection pose reached successfully."
        )

    except Exception as e:
        node.get_logger().error(f"Move failed: {e}")

    finally:
        if motion is not None:
            motion.shutdown()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()