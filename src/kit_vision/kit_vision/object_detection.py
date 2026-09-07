from time import perf_counter

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from kit_interfaces.msg import DetectedObject, DetectionArray
from kit_vision.realsense import ImgNode
from kit_vision.yolo_model import YoloModel
import cv2


# 최신 검출만 의미가 있으므로 과거 메시지를 쌓지 않는다.
DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)

PUBLISH_PERIOD_SEC = 0.1
MAX_SYNC_DELTA_SEC = 0.05


def polygon_depth_median(depth_frame, polygon) -> Optional[float]:
    """
    polygon 내부 depth 중앙값(mm).

    full-frame mask를 물체마다 만드는 대신 polygon bounding ROI만 잘라서 계산하므로
    1920x1080 입력에서도 CPU/메모리 복사를 크게 줄인다.
    """
    pts = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    if pts.shape[0] < 3:
        return None

    h, w = depth_frame.shape[:2]

    x0 = max(0, int(np.floor(pts[:, 0].min())))
    y0 = max(0, int(np.floor(pts[:, 1].min())))
    x1 = min(w - 1, int(np.ceil(pts[:, 0].max())))
    y1 = min(h - 1, int(np.ceil(pts[:, 1].max())))

    if x1 < x0 or y1 < y0:
        return None

    roi = depth_frame[y0 : y1 + 1, x0 : x1 + 1]
    if roi.size == 0:
        return None

    local_pts = pts.copy()
    local_pts[:, 0] -= x0
    local_pts[:, 1] -= y0
    local_pts = np.rint(local_pts).astype(np.int32)

    roi_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
    cv2.fillPoly(roi_mask, [local_pts], 1)

    valid = roi[roi_mask > 0]
    valid = valid[valid > 0]

    if valid.size == 0:
        return None

    return float(np.median(valid))


def pixel_to_camera(x, y, z, intrinsics):
    """픽셀 (x, y) + depth z(mm) -> 카메라 좌표 (X, Y, Z)(mm)."""
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    ppx = intrinsics["ppx"]
    ppy = intrinsics["ppy"]

    if fx == 0.0 or fy == 0.0:
        raise ValueError("Invalid camera intrinsics: fx/fy must be non-zero")

    return (
        (x - ppx) * z / fx,
        (y - ppy) * z / fy,
        z,
    )


class ObjectDetectionNode(ImgNode):
    """
    RealSense 구독 + YOLO 검출 노드.

    ROS executor:
        color/depth/camera_info 수신만 담당하며 계속 spin한다.

    inference worker:
        0.3초마다 '그 순간의 최신 동기화 프레임' 하나만 가져와 YOLO를 수행한다.

    따라서 YOLO가 느려져도 ROS image callback 큐를 따라가려고 과거 프레임을
    순차 처리하지 않는다. 처리 중 새 프레임이 여러 장 들어오면 중간 것은 버리고
    다음 추론에서는 가장 최신 프레임을 사용한다.
    """

    def __init__(self):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = YoloModel()
        self.publisher = self.create_publisher(DetectionArray, '/detection/objects', DETECTION_QOS)
        self.timer = self.create_timer(PUBLISH_PERIOD_SEC, self.timer_callback)
        self._last_stamp = None
        self._last_slow_warning_time = 0.0
        self.get_logger().info()
#        self.get_logger().info("ObjectDetectionNode initialized.")

    def timer_callback(self):
        callback_started = perf_counter()

        color, depth, intrinsics, header = self.img_node.get_snapshot()
        if color is None or depth is None or intrinsics is None or header is None:
            return  # 카메라 아직 준비 안 됨. 이번 틱은 건너뛴다.
        # 처리용 해상도: 1280x720 -> 640x360
        target_w = 640
        target_h = 360

        original_h, original_w = color.shape[:2]

        scale_x = target_w / original_w
        scale_y = target_h / original_h

        # RGB 축소
        color = cv2.resize(
            color,
            (target_w, target_h),
            interpolation=cv2.INTER_LINEAR,
        )

        # aligned depth 축소
        # depth 값 자체를 보간하면 안 되므로 NEAREST 사용
        depth = cv2.resize(
            depth,
            (target_w, target_h),
            interpolation=cv2.INTER_NEAREST,
        )

        # resize된 영상 좌표계에 맞게 camera intrinsics도 수정
        intrinsics = intrinsics.copy()

        intrinsics["fx"] *= scale_x
        intrinsics["fy"] *= scale_y
        intrinsics["ppx"] *= scale_x
        intrinsics["ppy"] *= scale_y

        self.bridge = CvBridge()
        self.model = YoloModel(imgsz=imgsz)

        self.publisher = self.create_publisher(
            DetectionArray,
            "/detection/objects",
            DETECTION_QOS,
        )

        # debug_view는 RealSense/YOLO를 다시 띄우지 않고 이 토픽만 본다.
        self.debug_publisher = self.create_publisher(
            Image,
            "/detection/debug_image",
            1,
        )

        self._last_processed_stamp_ns = None
        self._stop_event = threading.Event()

        self._worker = threading.Thread(
            target=self._inference_loop,
            name="kit_vision_inference",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            "ObjectDetectionNode initialized: "
            f"period={self.publish_period_sec:.3f}s, "
            f"sync_delta<={self.max_sync_delta_sec:.3f}s, "
            f"conf={self.conf_threshold:.2f}, imgsz={imgsz}"
        )

    def _inference_loop(self):
        while not self._stop_event.is_set():
            loop_started = time.monotonic()

            bundle = self.get_latest_frame(
                last_stamp_ns=self._last_processed_stamp_ns,
                max_sync_delta_sec=self.max_sync_delta_sec,
            )

            if bundle is not None:
                try:
                    self._process_bundle(bundle)
                    self._last_processed_stamp_ns = bundle.stamp_ns
                except Exception as exc:
                    self.get_logger().error(
                        f"Vision inference failed: {type(exc).__name__}: {exc}"
                    )

            elapsed = time.monotonic() - loop_started
            sleep_sec = max(0.0, self.publish_period_sec - elapsed)
            self._stop_event.wait(sleep_sec)

    def _process_bundle(self, bundle):
        # 선택된 최신 프레임만 numpy로 변환한다.
        color = self.bridge.imgmsg_to_cv2(
            bundle.color_msg,
            desired_encoding="bgr8",
        )
        depth = self.bridge.imgmsg_to_cv2(
            bundle.depth_msg,
            desired_encoding="passthrough",
        )

        inference_started = perf_counter()
        instances = self.model.infer(color)
        inference_ms = (perf_counter() - inference_started) * 1000.0

        objects = []
        for inst in instances:
            cz = mask_depth_median(depth, inst["mask"])
            if cz is None:
                continue

            cx, cy = inst["centroid_px"]
            x, y, z = pixel_to_camera(
                cx,
                cy,
                cz,
                bundle.intrinsics,
            )

            objects.append(
                DetectedObject(
                    class_name=inst["class_name"],
                    score=inst["score"],
                    camera_xyz=[float(x), float(y), float(z)],
                    masking_map=inst["polygon"],
                    centroid_px=[int(cx), int(cy)],
                )
            )
            debug_items.append((inst, cz))

        msg = DetectionArray()
        msg.header = bundle.color_msg.header
        msg.objects = objects
        self.publisher.publish(msg)

        finished = perf_counter()
        total_ms = (finished - callback_started) * 1000.0
        target_ms = PUBLISH_PERIOD_SEC * 1000.0
        if total_ms > target_ms and finished - self._last_slow_warning_time >= 5.0:
            self.get_logger().warning(
                "Detection missed target period: "
                f"total={total_ms:.1f}ms, inference={inference_ms:.1f}ms, "
                f"target={target_ms:.1f}ms"
            )
            self._last_slow_warning_time = finished


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(node.img_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.img_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo():
    depth = np.full((100, 100), 1000, dtype=np.uint16)
    polygon = [20, 20, 80, 20, 80, 80, 20, 80]
    median = polygon_depth_median(depth, polygon)
    assert median == 1000.0

    xyz = pixel_to_camera(
        50,
        50,
        1000,
        {"fx": 500.0, "fy": 500.0, "ppx": 50.0, "ppy": 50.0},
    )
    assert xyz == (0.0, 0.0, 1000)

    print("ok")


if __name__ == "__main__":
    _demo()
