import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from kit_interfaces.msg import DetectedObject, DetectionArray

from kit_vision.realsense import ImgNode
from kit_vision.yolo_model import YoloModel

# 검출 토픽은 상시 발행 + 최신 검출만 의미 있음 → BEST_EFFORT, depth=1 (02-interfaces.md §2.3)
# depth=1 이라 옛 프레임은 어차피 버려지고, 구독측이 max_age_sec 로 신선도를 앱 레벨에서
# 검사한다 — 한 틱 유실돼도 0.3s 뒤 다음 발행이 덮어쓴다. RELIABLE 의 재전송 보장이 할 일이 없다.
DETECTION_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

# 목표 2~5Hz. 0.3s ~= 3.3Hz.
PUBLISH_PERIOD_SEC = 0.3


def mask_depth_median(depth_frame, mask):
    """mask 영역의 depth 중앙값 (mm). 0 은 무효 측정값이라 제외. 유효값 없으면 None."""
    valid = depth_frame[mask > 0]
    valid = valid[valid > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def pixel_to_camera(x, y, z, intrinsics):
    """픽셀 (x, y) + depth z(mm) → 카메라 좌표 (X, Y, Z)(mm)."""
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    ppx, ppy = intrinsics["ppx"], intrinsics["ppy"]
    return ((x - ppx) * z / fx, (y - ppy) * z / fy, z)


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = YoloModel()
        self.publisher = self.create_publisher(DetectionArray, '/detection/objects', DETECTION_QOS)
        self.timer = self.create_timer(PUBLISH_PERIOD_SEC, self.timer_callback)
        self.get_logger().info("ObjectDetectionNode initialized.")

    def timer_callback(self):
        self.img_node.spin_once(timeout_sec=0.0)

        color = self.img_node.get_color_frame()
        depth = self.img_node.get_depth_frame()
        intrinsics = self.img_node.get_camera_intrinsic()
        header = self.img_node.get_color_frame_header()
        if color is None or depth is None or intrinsics is None or header is None:
            return  # 카메라 아직 준비 안 됨. 이번 틱은 건너뛴다.

        objects = []
        for inst in self.model.infer(color):
            cz = mask_depth_median(depth, inst["mask"])
            if cz is None:
                continue  # 계약: depth 무효인 검출은 발행하지 않는다 (02-interfaces.md §2.3)

            cx, cy = inst["centroid_px"]
            x, y, z = pixel_to_camera(cx, cy, cz, intrinsics)
            objects.append(DetectedObject(
                class_name=inst["class_name"],
                score=inst["score"],
                camera_xyz=[x, y, z],
                masking_map=inst["polygon"],
                centroid_px=[cx, cy],
            ))

        msg = DetectionArray()
        msg.header = header
        msg.objects = objects
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _demo():
    # 노드/카메라/모델 없이도 도는 순수 함수 self-check.
    depth = np.zeros((10, 10), dtype=np.uint16)
    depth[2:5, 2:5] = 500  # mm
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[2:5, 2:5] = 1
    assert mask_depth_median(depth, mask) == 500.0
    assert mask_depth_median(np.zeros((10, 10), dtype=np.uint16), mask) is None

    intr = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 50.0}
    x, y, z = pixel_to_camera(50, 50, 500.0, intr)
    assert (x, y, z) == (0.0, 0.0, 500.0)
    x, y, z = pixel_to_camera(150, 50, 200.0, intr)
    assert np.isclose(x, 200.0)
    print("ok")


if __name__ == '__main__':
    _demo()
