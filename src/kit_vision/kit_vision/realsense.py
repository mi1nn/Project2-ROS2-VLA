from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# RealSense 드라이버는 이미지 토픽을 BEST_EFFORT 로 발행한다. RELIABLE 로 구독하면
# QoS 불일치로 콜백이 아예 안 불릴 수 있다 (reference/subscriber_sourcecode/subscriber_img.py).
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


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
        self._img_exec.spin_once(timeout_sec=timeout_sec)

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
