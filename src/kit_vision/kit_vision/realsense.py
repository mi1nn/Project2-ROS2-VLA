from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# RealSense 드라이버는 이미지 토픽을 BEST_EFFORT 로 발행한다. RELIABLE 로 구독하면
# QoS 불일치로 콜백이 아예 안 불릴 수 있다 (reference/subscriber_sourcecode/subscriber_img.py).
# depth=1: eye-in-hand 라 의미 있는 건 언제나 "지금" 프레임뿐이다. 큐를 쌓아두면
# 팔이 움직인 뒤에 이동 전 프레임을 꺼내 쓰게 되고 그건 통째로 틀린 좌표다.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

# 구독 3개(color/depth/camera_info) × depth=1 → 한 번에 대기할 수 있는 콜백 수의 상한.
_DRAIN_LIMIT = 6


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.color_frame_header = None
        self.depth_frame = None
        self.intrinsics = None
        self.color_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, IMAGE_QOS)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, IMAGE_QOS)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, IMAGE_QOS)
        self.get_logger().info("Waiting for client's call...")
        self._img_exec = SingleThreadedExecutor()
        self._img_exec.add_node(self)

    def spin_once(self, timeout_sec=0.1):
        """대기 중인 이미지 콜백을 전부 비운다.

        Executor.spin_once 는 콜백을 하나만 처리한다. 구독이 3개라 그대로 쓰면
        color 갱신률이 호출률의 1/3 로 떨어지고 나머지는 큐에 밀린다.
        """
        self._img_exec.spin_once(timeout_sec=timeout_sec)
        for _ in range(_DRAIN_LIMIT):
            self._img_exec.spin_once(timeout_sec=0.0)

    def camera_info_callback(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_header = msg.header

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_color_frame(self):
        return self.color_frame

    def get_color_frame_header(self):
        # DetectionArray.header 에 그대로 옮길 원본 std_msgs/Header.
        # (발행 시각이 아니라 이 프레임이 찍힌 시각 — 최신성 판정의 근거)
        return self.color_frame_header

    def get_depth_frame(self):
        return self.depth_frame

    def get_camera_intrinsic(self):
        return self.intrinsics
