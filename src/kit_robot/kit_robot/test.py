import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "current_pose_test",
        namespace=ROBOT_ID
    )

    DR_init.__dsr__node = node

    # 중요: node 설정 후 import
    from DSR_ROBOT2 import get_current_posx

    try:
        pos = get_current_posx()
        print("Current TCP pose:")
        print(pos)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
