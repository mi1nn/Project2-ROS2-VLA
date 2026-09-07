import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from kit_interfaces.msg import DetectedObject, DetectionArray
from kit_vision.realsense import ImgNode
from kit_vision.yolo_model import YoloModel


# 검출 결과도 과거 메시지를 쌓지 않고 최신 결과가 중요하다.
DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)

MAX_SYNC_DELTA_SEC = 0.05
NO_FRAME_WAIT_SEC = 0.001
PERF_LOG_EVERY_N = 30

# 추론 사이클의 하한 주기. 목표 2~5Hz, 0.3s ~= 3.3Hz.
# 추론 본체는 GPU 로 돌지만(compose.yaml 이 nvidia 디바이스를 예약한다) cv_bridge 변환·
# 전처리·NMS·polygon depth 는 CPU 다. free-run 으로 두면 이 CPU 구간이 코어를 계속 물고
# 있어서, 같은 호스트에서 도는 realsense2_camera USB 드라이버 스레드가 스케줄링을 못 받고
# "Incomplete video frame / Frame Corrupted" 로 이어진다 — 영상 딜레이의 원인이었다.
# yolo_model.py 의 torch.set_num_threads(2) 와 같은 목적, 다른 축(스레드 수 vs 점유율).
# 추론이 0.3s 보다 느리면 이 하한은 그냥 통과한다(추가 지연 없음).
MIN_CYCLE_SEC = 0.3


def polygon_depth_median(
    depth_frame,
    polygon,
) -> Optional[float]:
    """
    polygon 내부의 유효 depth 중앙값을 계산한다.

    full-resolution mask를 매 객체마다 만들지 않고,
    polygon bounding ROI 안에서만 mask/depth를 계산한다.
    """
    pts = np.asarray(
        polygon,
        dtype=np.float32,
    ).reshape(-1, 2)

    if pts.shape[0] < 3:
        return None

    h, w = depth_frame.shape[:2]

    x0 = max(
        0,
        int(np.floor(pts[:, 0].min())),
    )
    y0 = max(
        0,
        int(np.floor(pts[:, 1].min())),
    )
    x1 = min(
        w - 1,
        int(np.ceil(pts[:, 0].max())),
    )
    y1 = min(
        h - 1,
        int(np.ceil(pts[:, 1].max())),
    )

    if x1 < x0 or y1 < y0:
        return None

    roi = depth_frame[
        y0:y1 + 1,
        x0:x1 + 1,
    ]

    if roi.size == 0:
        return None

    local_pts = pts.copy()
    local_pts[:, 0] -= x0
    local_pts[:, 1] -= y0
    local_pts = np.rint(
        local_pts
    ).astype(np.int32)

    roi_mask = np.zeros(
        roi.shape[:2],
        dtype=np.uint8,
    )

    cv2.fillPoly(
        roi_mask,
        [local_pts],
        1,
    )

    valid = roi[roi_mask > 0]
    valid = valid[valid > 0]

    if valid.size == 0:
        return None

    return float(np.median(valid))


def pixel_to_camera(
    x,
    y,
    z,
    intrinsics,
):
    """
    픽셀 (x, y) + depth z(mm)
    -> 카메라 좌표 (X, Y, Z)(mm)
    """
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    ppx = intrinsics["ppx"]
    ppy = intrinsics["ppy"]

    if fx == 0.0 or fy == 0.0:
        raise ValueError(
            "Invalid camera intrinsics: fx/fy must be non-zero"
        )

    return (
        (x - ppx) * z / fx,
        (y - ppy) * z / fy,
        z,
    )


class ObjectDetectionNode(ImgNode):
    """
    RealSense subscriber + YOLO detection node.

    ROS executor:
        color/depth/camera_info callback을 계속 처리한다.
        callback은 최신 ROS 메시지 참조만 저장한다.

    inference worker:
        인위적인 0.3초 주기가 없다.
        YOLO 처리가 끝나는 즉시 그 시점의 최신 프레임을 가져간다.

    예:
        YOLO가 frame 1을 처리하는 동안
        frame 2, 3, 4, 5가 들어오면
        다음 추론은 frame 5를 사용한다.

    따라서 처리 FPS보다 카메라 FPS가 높아도
    과거 frame backlog가 누적되지 않는다.
    """

    def __init__(self):
        super().__init__("object_detection_node")

        self.declare_parameter(
            "max_sync_delta_sec",
            MAX_SYNC_DELTA_SEC,
        )
        self.declare_parameter(
            "conf_threshold",
            0.5,
        )
        self.declare_parameter(
            "imgsz",
            640,
        )
        self.declare_parameter(
            "perf_log_every_n",
            PERF_LOG_EVERY_N,
        )
        self.declare_parameter(
            "min_cycle_sec",
            MIN_CYCLE_SEC,
        )

        self.max_sync_delta_sec = float(
            self.get_parameter(
                "max_sync_delta_sec"
            ).value
        )

        self.conf_threshold = float(
            self.get_parameter(
                "conf_threshold"
            ).value
        )

        imgsz = int(
            self.get_parameter(
                "imgsz"
            ).value
        )

        self.perf_log_every_n = max(
            1,
            int(
                self.get_parameter(
                    "perf_log_every_n"
                ).value
            ),
        )

        self.min_cycle_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "min_cycle_sec"
                ).value
            ),
        )

        self.bridge = CvBridge()
        self.model = YoloModel(
            imgsz=imgsz,
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            "/detection/objects",
            DETECTION_QOS,
        )

        # debug_view는 RealSense와 YOLO를 다시 실행하지 않는다.
        # 이 토픽만 구독한다.
        self.debug_publisher = self.create_publisher(
            Image,
            "/detection/debug_image",
            1,
        )

        self._last_processed_stamp_ns = None
        self._stop_event = threading.Event()

        self._processed_count = 0
        self._perf_window_started = time.monotonic()

        self._worker = threading.Thread(
            target=self._inference_loop,
            name="kit_vision_inference",
            daemon=True,
        )
        self._worker.start()

        max_hz = (
            1.0 / self.min_cycle_sec
            if self.min_cycle_sec > 0
            else float("inf")
        )

        self.get_logger().info(
            "ObjectDetectionNode initialized: "
            f"min_cycle={self.min_cycle_sec:.3f}s (<={max_hz:.1f}Hz), "
            f"sync_delta<={self.max_sync_delta_sec:.3f}s, "
            f"conf={self.conf_threshold:.2f}, "
            f"imgsz={imgsz}"
        )

    def _inference_loop(self):
        while not self._stop_event.is_set():
            cycle_started = time.monotonic()

            bundle = self.get_latest_frame(
                last_stamp_ns=self._last_processed_stamp_ns,
                max_sync_delta_sec=self.max_sync_delta_sec,
            )

            if bundle is None:
                # 새로운 동기화 frame이 없을 때만 아주 짧게 대기한다.
                # 이 대기는 FPS 제한용이 아니라 CPU busy-loop 방지용이다.
                self._stop_event.wait(
                    NO_FRAME_WAIT_SEC
                )
                continue

            try:
                self._process_bundle(bundle)

                self._last_processed_stamp_ns = (
                    bundle.stamp_ns
                )

            except Exception as exc:
                self.get_logger().error(
                    "Vision inference failed: "
                    f"{type(exc).__name__}: {exc}"
                )

            # 처리가 빨랐으면 남은 시간만큼 쉰다. 이미 느렸으면 0 이라 그대로 진행한다.
            elapsed = time.monotonic() - cycle_started
            self._stop_event.wait(
                max(0.0, self.min_cycle_sec - elapsed)
            )

    def _process_bundle(self, bundle):
        cycle_started = time.perf_counter()

        # -------------------------------------------------
        # 1. ROS Image -> numpy
        # -------------------------------------------------
        bridge_started = time.perf_counter()

        color = self.bridge.imgmsg_to_cv2(
            bundle.color_msg,
            desired_encoding="bgr8",
        )

        depth = self.bridge.imgmsg_to_cv2(
            bundle.depth_msg,
            desired_encoding="passthrough",
        )

        bridge_ms = (
            time.perf_counter()
            - bridge_started
        ) * 1000.0

        # -------------------------------------------------
        # 2. YOLO
        # -------------------------------------------------
        infer_started = time.perf_counter()

        instances = self.model.infer(
            color,
            conf_threshold=self.conf_threshold,
        )

        infer_ms = (
            time.perf_counter()
            - infer_started
        ) * 1000.0

        # -------------------------------------------------
        # 3. polygon depth + 3D position
        # -------------------------------------------------
        post_started = time.perf_counter()

        objects = []
        debug_items = []

        for inst in instances:
            cz = polygon_depth_median(
                depth,
                inst["polygon"],
            )

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
                    camera_xyz=[
                        float(x),
                        float(y),
                        float(z),
                    ],
                    masking_map=inst["polygon"],
                    centroid_px=[
                        int(cx),
                        int(cy),
                    ],
                )
            )

            debug_items.append(
                (inst, cz)
            )

        msg = DetectionArray()
        msg.header = bundle.color_msg.header
        msg.objects = objects

        self.publisher.publish(msg)

        post_ms = (
            time.perf_counter()
            - post_started
        ) * 1000.0

        # -------------------------------------------------
        # 4. debug image
        # -------------------------------------------------
        debug_started = time.perf_counter()

        if (
            self.debug_publisher
            .get_subscription_count()
            > 0
        ):
            self._publish_debug_image(
                color,
                bundle.color_msg.header,
                debug_items,
            )

        debug_ms = (
            time.perf_counter()
            - debug_started
        ) * 1000.0

        total_ms = (
            time.perf_counter()
            - cycle_started
        ) * 1000.0

        self._processed_count += 1

        if (
            self._processed_count
            % self.perf_log_every_n
            == 0
        ):
            now = time.monotonic()

            window_sec = (
                now
                - self._perf_window_started
            )

            fps = (
                self.perf_log_every_n
                / window_sec
                if window_sec > 0
                else 0.0
            )

            self.get_logger().info(
                "[PERF] "
                f"fps={fps:.2f}, "
                f"bridge={bridge_ms:.1f}ms, "
                f"yolo={infer_ms:.1f}ms, "
                f"post={post_ms:.1f}ms, "
                f"debug={debug_ms:.1f}ms, "
                f"total={total_ms:.1f}ms, "
                f"objects={len(objects)}"
            )

            self._perf_window_started = now

    def _publish_debug_image(
        self,
        color,
        header,
        debug_items,
    ):
        debug = color.copy()

        for inst, depth_mm in debug_items:
            pts = np.asarray(
                inst["polygon"],
                dtype=np.float32,
            ).reshape(-1, 2)

            pts_i = np.rint(
                pts
            ).astype(np.int32)

            cv2.polylines(
                debug,
                [pts_i],
                isClosed=True,
                color=(255, 255, 255),
                thickness=2,
            )

            cx, cy = inst["centroid_px"]

            cv2.circle(
                debug,
                (int(cx), int(cy)),
                4,
                (255, 255, 255),
                -1,
            )

            label = (
                f'{inst["class_name"]} '
                f'{inst["score"]:.2f} '
                f'{depth_mm:.0f}mm'
            )

            cv2.putText(
                debug,
                label,
                (
                    int(cx) + 6,
                    int(cy) - 6,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(
            debug,
            encoding="bgr8",
        )

        debug_msg.header = header

        self.debug_publisher.publish(
            debug_msg
        )

    def destroy_node(self):
        self._stop_event.set()

        if (
            hasattr(self, "_worker")
            and self._worker.is_alive()
        ):
            self._worker.join(
                timeout=3.0
            )

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()

    try:
        # executor는 RealSense callbacks에 집중한다.
        # YOLO는 별도 worker에서 최대 처리속도로 실행한다.
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def _demo():
    # 노드/카메라/모델 없이 도는 순수 함수 self-check.
    depth = np.zeros((10, 10), dtype=np.uint16)
    depth[2:8, 2:8] = 500  # mm
    square = [2, 2, 7, 2, 7, 7, 2, 7]
    assert polygon_depth_median(depth, square) == 500.0

    # depth 0(무효 측정값)만 있으면 None
    assert polygon_depth_median(
        np.zeros((10, 10), dtype=np.uint16), square
    ) is None
    # 점 3개 미만이면 None
    assert polygon_depth_median(depth, [2, 2, 7, 2]) is None
    # 프레임 밖으로 나간 polygon 도 클리핑되어 죽지 않는다
    assert polygon_depth_median(depth, [-50, -50, 60, 2, 7, 60]) is not None

    intr = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 50.0}
    assert pixel_to_camera(50, 50, 500.0, intr) == (0.0, 0.0, 500.0)
    x, y, z = pixel_to_camera(150, 50, 200.0, intr)
    assert np.isclose(x, 200.0) and np.isclose(y, 0.0) and z == 200.0

    try:
        pixel_to_camera(0, 0, 1.0, {"fx": 0.0, "fy": 1.0, "ppx": 0.0, "ppy": 0.0})
    except ValueError:
        pass
    else:
        raise AssertionError("fx=0 은 ValueError 여야 한다")

    print("ok")


if __name__ == "__main__":
    _demo()
