import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from kit_vision.realsense import IMAGE_QOS


class DebugViewNode(Node):
    """
    검출 결과 시각화 전용 노드.

    중요:
    - RealSense 토픽 직접 구독 안 함
    - YoloModel 생성 안 함
    - /detection/debug_image만 표시함
    """

    def __init__(self):
        super().__init__("debug_view_node")

        self.bridge = CvBridge()
        self.latest_frame = None

        self.subscription = self.create_subscription(
            Image,
            "/detection/debug_image",
            self._image_callback,
            IMAGE_QOS,
        )

        self.get_logger().info(
            "Waiting for /detection/debug_image ..."
        )

    def _image_callback(self, msg: Image):
        self.latest_frame = (
            self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        )


def main(args=None):
    rclpy.init(args=args)

    node = DebugViewNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

            if node.latest_frame is not None:
                cv2.imshow(
                    "kit_vision debug (q to quit)",
                    node.latest_frame,
                )

            if (
                cv2.waitKey(1)
                & 0xFF
                == ord("q")
            ):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
