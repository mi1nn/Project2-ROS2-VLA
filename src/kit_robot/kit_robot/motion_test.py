import time
import DR_init
import rclpy

from .motion import Motion

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "motion_test",
        namespace=ROBOT_ID,
    )
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    motion = Motion(node)

    motions = [
        ("home", motion.home),
        ("pick_camera", motion.pick_camera),
        ("place_camera", motion.place_camera),
    ]

    node.get_logger().info("motion library test start")

    for name, move_function in motions:
        position = motion.positions[name]["pos"]

        answer = input(
            f"Move to {name} {position}? [y/N]: "
        ).strip().lower()

        if answer != "y":
            node.get_logger().warning("motion test stopped")
            break

        node.get_logger().info(f"move to {name}")

        result = move_function()

        if result != 0:
            raise RuntimeError(
                f"{name} motion failed: result={result}"
            )

        node.get_logger().info(f"complete move {name}")
        time.sleep(2.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()