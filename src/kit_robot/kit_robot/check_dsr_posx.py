import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "check_dsr_posx",
        namespace=ROBOT_ID,
    )

    # DSR_ROBOT2 import 전에 반드시 설정
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            DR_BASE,
        )

        node.get_logger().info(
            "Reading current TCP pose with get_current_posx(ref=DR_BASE)..."
        )

        pose, solution_space = get_current_posx(
            ref=DR_BASE
        )

        if pose is None:
            raise RuntimeError(
                "get_current_posx returned None"
            )

        values = [
            float(pose[i])
            for i in range(6)
        ]

        print()
        print("========================================")
        print("DSR get_current_posx(ref=DR_BASE)")
        print("========================================")
        print(f"X  = {values[0]:.6f} mm")
        print(f"Y  = {values[1]:.6f} mm")
        print(f"Z  = {values[2]:.6f} mm")
        print(f"RX = {values[3]:.6f} deg")
        print(f"RY = {values[4]:.6f} deg")
        print(f"RZ = {values[5]:.6f} deg")
        print(f"solution_space = {solution_space}")
        print()
        print("raw pose:")
        print(values)
        print("========================================")
        print()

    except Exception as exc:
        node.get_logger().error(
            f"Failed to read current pose: {exc}"
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
