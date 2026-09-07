from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


# 실시간 영상은 "모든 프레임 보존"보다 "최신 프레임 유지"가 중요하다.
# depth=10으로 두면 소비가 느릴 때 오래된 프레임이 큐에 남아 지연이 커질 수 있으므로
# depth=1로 두고, 처리 속도보다 카메라 FPS가 높으면 과거 프레임을 버린다.
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
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.color_frame_header = None
        self.depth_frame = None
        self.intrinsics = None
        self._frame_lock = Lock()
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
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, IMAGE_QOS)
        self.get_logger().info("Waiting for client's call...")

    def spin_once(self, timeout_sec=0.1):
        """독립 실행 도구용 호환 API. detection 노드에서는 호출하지 않는다."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def camera_info_callback(self, msg):
        intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}
        with self._frame_lock:
            self.intrinsics = intrinsics

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._frame_lock:
            self.color_frame = frame
            self.color_frame_header = msg.header

    def depth_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with self._frame_lock:
            self.depth_frame = frame

    def get_snapshot(self):
        """검출 한 주기에서 사용할 최신 카메라 데이터를 일관되게 반환한다."""
        with self._frame_lock:
            return (
                self.color_frame,
                self.depth_frame,
                self.intrinsics,
                self.color_frame_header,
            )

    def get_color_frame(self):
        with self._frame_lock:
            return self.color_frame

    def get_color_frame_header(self):
        # DetectionArray.header 에 그대로 옮길 원본 std_msgs/Header.
        # (발행 시각이 아니라 이 프레임이 찍힌 시각 — 최신성 판정의 근거)
        with self._frame_lock:
            return self.color_frame_header

    def get_depth_frame(self):
        with self._frame_lock:
            return self.depth_frame

    def get_camera_intrinsic(self):
        with self._frame_lock:
            return self.intrinsics
