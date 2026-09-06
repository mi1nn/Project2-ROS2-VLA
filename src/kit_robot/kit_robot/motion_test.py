import time
import DR_init
import rclpy

from .motion import Motion

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
PICK_TARGET_POSE = [300, -100, 300.0, 0, 180.0, 0]
PICK_VEL = 100.0
PICK_ACC = 200.0
APPROACH_HEIGHT = 100.0


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

    if not motion.rg.connected:
        raise ConnectionError("RG2 is not connected")

    node.get_logger().info("pick motion test start")

    motion.home()

    target_pose = PICK_TARGET_POSE.copy()
    approach_pose = target_pose.copy()
    approach_pose[2] += APPROACH_HEIGHT

    print(f"pick target   : {target_pose}")
    print(f"approach pose : {approach_pose}")

    answer = input("Run pick motion? [y/N]: ").strip().lower()
    if answer != "y":
        node.get_logger().warning("pick motion test cancelled")
        rclpy.shutdown()
        return

    success = motion.pick(
        target_pose,
        vel=PICK_VEL,
        acc=PICK_ACC,
        approach_height=APPROACH_HEIGHT,
    )

    if success:
        node.get_logger().info("pick succeeded")
    else:
        node.get_logger().warning("pick failed: gripper width <= 13 mm")

    answer = input("Open gripper after test? [y/N]: ").strip().lower()
    if answer == "y":
        motion.rg.open_gripper()
        time.sleep(2.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
