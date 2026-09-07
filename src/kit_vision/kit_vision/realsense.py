from dataclasses import dataclass
from threading import Lock
from typing import Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


# 실시간 비전에서는 과거 프레임 보존보다 최신 프레임 유지가 중요하다.
# 처리 속도가 카메라 FPS보다 느릴 경우 오래된 프레임을 큐에 쌓지 않는다.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)


def _stamp_to_ns(msg) -> int:
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class FrameBundle:
    color_msg: Image
    depth_msg: Image
    intrinsics: dict
    stamp_ns: int


class ImgNode(Node):
    """
    RealSense ROS 토픽의 최신 메시지만 보관한다.

    핵심:
    - callback 안에서는 cv_bridge 변환을 하지 않는다.
    - color/depth/camera_info의 최신 ROS 메시지 참조만 저장한다.
    - 실제 numpy 변환은 YOLO가 처리할 최신 프레임 1장에 대해서만 수행한다.
    - 별도의 executor/spin_once를 만들지 않는다.
    """

    def __init__(self, node_name: str = "img_node"):
        super().__init__(node_name)

        self._frame_lock = Lock()
        self._color_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._intrinsics: Optional[dict] = None

        self.color_subscription = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self._color_callback,
            IMAGE_QOS,
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self._depth_callback,
            IMAGE_QOS,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self._camera_info_callback,
            IMAGE_QOS,
        )

        self.get_logger().info(
            "Waiting for RealSense color/depth/camera_info topics..."
        )

    def _camera_info_callback(self, msg: CameraInfo):
        intrinsics = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "ppx": float(msg.k[2]),
            "ppy": float(msg.k[5]),
        }

        with self._frame_lock:
            self._intrinsics = intrinsics

    def _color_callback(self, msg: Image):
        with self._frame_lock:
            self._color_msg = msg

    def _depth_callback(self, msg: Image):
        with self._frame_lock:
            self._depth_msg = msg

    def get_latest_frame(
        self,
        last_stamp_ns: Optional[int] = None,
        max_sync_delta_sec: float = 0.05,
    ) -> Optional[FrameBundle]:
        """
        현재 보관 중인 가장 최신 color/depth 쌍을 반환한다.

        - color/depth timestamp 차이가 max_sync_delta_sec보다 크면 None
        - last_stamp_ns와 같은 color frame이면 이미 처리한 프레임이므로 None
        """
        with self._frame_lock:
            color_msg = self._color_msg
            depth_msg = self._depth_msg
            intrinsics = (
                None
                if self._intrinsics is None
                else dict(self._intrinsics)
            )

        if color_msg is None or depth_msg is None or intrinsics is None:
            return None

        color_ns = _stamp_to_ns(color_msg)
        depth_ns = _stamp_to_ns(depth_msg)

        if last_stamp_ns is not None and color_ns == last_stamp_ns:
            return None

        max_delta_ns = int(max_sync_delta_sec * 1_000_000_000)

        if abs(color_ns - depth_ns) > max_delta_ns:
            return None

        return FrameBundle(
            color_msg=color_msg,
            depth_msg=depth_msg,
            intrinsics=intrinsics,
            stamp_ns=color_ns,
        )
