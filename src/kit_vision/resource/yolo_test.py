import cv2
import json
import math
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, String
from ultralytics import YOLO

from object_detection.realsense import ImgNode


# =========================================================
# 설정
# =========================================================

MODEL_PATH = "/Users/min/orca/projects/Project2-ROS2-VLA/src/kit_detection_test/model/Yolo26_best.pt"

CONF = 0.25
IMGSZ = 960

# 중심점 주변 depth 영역
DEPTH_WINDOW = 5


class YoloTestNode(Node):

    def __init__(self):
        super().__init__("yolo_test")

        # =================================================
        # RealSense
        # =================================================
        self.img_node = ImgNode()

        # =================================================
        # YOLO
        # =================================================
        self.model = YOLO(MODEL_PATH)

        self.get_logger().info(
            f"YOLO model loaded: {MODEL_PATH}"
        )

        self.get_logger().info(
            f"Classes: {self.model.names}"
        )

        # =================================================
        # ROS Publishers
        # =================================================

        # detection 개수
        self.count_pub = self.create_publisher(
            Int32,
            "/yolo/detection_count",
            10
        )

        # 전체 detection 정보
        self.detection_pub = self.create_publisher(
            String,
            "/yolo/detections",
            10
        )

        # 중심점 / depth / gripper angle
        self.center_pub = self.create_publisher(
            String,
            "/yolo/centers",
            10
        )

        self.get_logger().info(
            "Publishing: /yolo/detection_count"
        )

        self.get_logger().info(
            "Publishing: /yolo/detections"
        )

        self.get_logger().info(
            "Publishing: /yolo/centers"
        )

        self.get_logger().info(
            "Waiting for D435i image..."
        )

    # =====================================================
    # 중심 주변 Depth median
    # =====================================================

    def get_depth_at_center(self, cx, cy, win=DEPTH_WINDOW):

        depth_frame = self.img_node.get_depth_frame()

        if depth_frame is None:
            return None

        h, w = depth_frame.shape[:2]

        if not (0 <= cx < w and 0 <= cy < h):
            return None

        x0 = max(0, cx - win)
        x1 = min(w, cx + win + 1)

        y0 = max(0, cy - win)
        y1 = min(h, cy + win + 1)

        patch = depth_frame[
            y0:y1,
            x0:x1
        ]

        # 0 = invalid depth
        valid = patch[patch > 0]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    # =====================================================
    # 각도 0~180도 정규화
    # =====================================================

    def normalize_angle(self, angle):

        angle = angle % 180.0

        if angle < 0:
            angle += 180.0

        return angle

    # =====================================================
    # Polygon의 긴 축 / 짧은 축 계산
    #
    # major_angle:
    #   물체의 긴 방향
    #
    # minor_angle:
    #   물체의 짧은 방향
    #
    # gripper_angle:
    #   그리퍼가 닫히는 방향으로 사용할 각도
    #   = minor_angle
    # =====================================================

    def get_object_orientation(self, polygon):

        if polygon is None or len(polygon) < 3:
            return None

        polygon_np = np.array(
            polygon,
            dtype=np.float32
        )

        # Polygon을 감싸는 최소 회전 사각형
        rect = cv2.minAreaRect(polygon_np)

        box = cv2.boxPoints(rect)

        # 4개 꼭짓점
        p0 = box[0]
        p1 = box[1]
        p2 = box[2]

        # 인접한 두 변
        edge1 = p1 - p0
        edge2 = p2 - p1

        length1 = np.linalg.norm(edge1)
        length2 = np.linalg.norm(edge2)

        # 긴 축 / 짧은 축 결정
        if length1 >= length2:

            major_vector = edge1
            minor_vector = edge2

            major_length = length1
            minor_length = length2

        else:

            major_vector = edge2
            minor_vector = edge1

            major_length = length2
            minor_length = length1

        # 영상 좌표계상의 각도
        major_angle = math.degrees(
            math.atan2(
                major_vector[1],
                major_vector[0]
            )
        )

        minor_angle = math.degrees(
            math.atan2(
                minor_vector[1],
                minor_vector[0]
            )
        )

        major_angle = self.normalize_angle(
            major_angle
        )

        minor_angle = self.normalize_angle(
            minor_angle
        )

        return {
            "major_angle": major_angle,
            "minor_angle": minor_angle,

            # 평행 그리퍼 closing 방향
            "gripper_angle": minor_angle,

            "major_length": float(major_length),
            "minor_length": float(minor_length),

            "box": box
        }

    # =====================================================
    # 영상에 축 그리기
    # =====================================================

    def draw_axis(
        self,
        image,
        cx,
        cy,
        angle_deg,
        length,
        color,
        thickness=3
    ):

        angle_rad = math.radians(
            angle_deg
        )

        dx = int(
            length * math.cos(angle_rad)
        )

        dy = int(
            length * math.sin(angle_rad)
        )

        pt1 = (
            cx - dx,
            cy - dy
        )

        pt2 = (
            cx + dx,
            cy + dy
        )

        cv2.line(
            image,
            pt1,
            pt2,
            color,
            thickness
        )

    # =====================================================
    # Main Loop
    # =====================================================

    def run(self):

        while rclpy.ok():

            # =================================================
            # RealSense callback 처리
            # =================================================
            self.img_node.spin_once(
                timeout_sec=0.1
            )

            frame = self.img_node.get_color_frame()

            if frame is None:
                continue

            # =================================================
            # YOLO inference
            # =================================================
            results = self.model.predict(
                source=frame,
                imgsz=IMGSZ,
                conf=CONF,
                device=0,
                verbose=False
            )

            result = results[0]

            detections = []
            centers = []

            # =================================================
            # Detection parsing
            # =================================================
            if result.boxes is not None:

                for i, box in enumerate(result.boxes):

                    # -----------------------------------------
                    # Class
                    # -----------------------------------------
                    class_id = int(
                        box.cls[0]
                    )

                    confidence = float(
                        box.conf[0]
                    )

                    class_name = (
                        result.names[class_id]
                    )

                    # -----------------------------------------
                    # Bounding Box
                    # -----------------------------------------
                    x1, y1, x2, y2 = (
                        box.xyxy[0]
                        .cpu()
                        .tolist()
                    )

                    detection = {

                        "class_id": class_id,

                        "class_name": class_name,

                        "confidence": round(
                            confidence,
                            4
                        ),

                        "bbox": {
                            "x1": round(x1, 2),
                            "y1": round(y1, 2),
                            "x2": round(x2, 2),
                            "y2": round(y2, 2)
                        }
                    }

                    # 기본값
                    center_x = None
                    center_y = None

                    depth = None

                    orientation = None

                    # =================================================
                    # Segmentation
                    # =================================================
                    if (
                        result.masks is not None
                        and i < len(result.masks.xy)
                    ):

                        polygon = (
                            result.masks.xy[i]
                        )

                        detection["polygon"] = (
                            polygon.tolist()
                        )

                        polygon_np = np.array(
                            polygon,
                            dtype=np.float32
                        )

                        # =============================================
                        # Polygon 면적 중심
                        # =============================================
                        moments = cv2.moments(
                            polygon_np
                        )

                        if moments["m00"] != 0:

                            center_x = int(
                                moments["m10"]
                                / moments["m00"]
                            )

                            center_y = int(
                                moments["m01"]
                                / moments["m00"]
                            )

                            # =========================================
                            # Depth
                            # =========================================
                            depth = (
                                self.get_depth_at_center(
                                    center_x,
                                    center_y
                                )
                            )

                        # =============================================
                        # Orientation
                        # =============================================
                        orientation = (
                            self.get_object_orientation(
                                polygon
                            )
                        )

                    else:

                        detection["polygon"] = []

                    # =================================================
                    # Center
                    # =================================================
                    if (
                        center_x is not None
                        and center_y is not None
                    ):

                        detection["center"] = {

                            "x": center_x,

                            "y": center_y,

                            "depth": depth
                        }

                    else:

                        detection["center"] = None

                    # =================================================
                    # Orientation
                    # =================================================
                    if orientation is not None:

                        detection["orientation"] = {

                            "major_angle": round(
                                orientation[
                                    "major_angle"
                                ],
                                2
                            ),

                            "minor_angle": round(
                                orientation[
                                    "minor_angle"
                                ],
                                2
                            ),

                            "gripper_angle": round(
                                orientation[
                                    "gripper_angle"
                                ],
                                2
                            ),

                            "major_length": round(
                                orientation[
                                    "major_length"
                                ],
                                2
                            ),

                            "minor_length": round(
                                orientation[
                                    "minor_length"
                                ],
                                2
                            )
                        }

                    else:

                        detection[
                            "orientation"
                        ] = None

                    detections.append(
                        detection
                    )

                    # =================================================
                    # Center topic용 간단 정보
                    # =================================================
                    if (
                        center_x is not None
                        and center_y is not None
                    ):

                        center_data = {

                            "class_id": class_id,

                            "class_name": class_name,

                            "confidence": round(
                                confidence,
                                4
                            ),

                            "x": center_x,

                            "y": center_y,

                            "depth": depth
                        }

                        if orientation is not None:

                            center_data[
                                "major_angle"
                            ] = round(
                                orientation[
                                    "major_angle"
                                ],
                                2
                            )

                            center_data[
                                "minor_angle"
                            ] = round(
                                orientation[
                                    "minor_angle"
                                ],
                                2
                            )

                            center_data[
                                "gripper_angle"
                            ] = round(
                                orientation[
                                    "gripper_angle"
                                ],
                                2
                            )

                        centers.append(
                            center_data
                        )

            # =================================================
            # Detection Count
            # =================================================
            count = len(detections)

            # =================================================
            # Terminal Log
            # =================================================
            self.get_logger().info(
                f"Detection count: {count}"
            )

            for det in detections:

                center = det["center"]

                orientation = det[
                    "orientation"
                ]

                if center is not None:

                    if center["depth"] is not None:

                        depth_text = (
                            f"{center['depth']:.1f}"
                        )

                    else:

                        depth_text = "N/A"

                    if orientation is not None:

                        angle_text = (
                            f"{orientation['gripper_angle']:.1f}"
                        )

                    else:

                        angle_text = "N/A"

                    self.get_logger().info(

                        f"  {det['class_name']} "

                        f"(id={det['class_id']}, "

                        f"conf={det['confidence']:.3f}) "

                        f"center=({center['x']}, "
                        f"{center['y']}), "

                        f"depth={depth_text}, "

                        f"gripper_angle={angle_text} deg"
                    )

            # =================================================
            # /yolo/detection_count
            # =================================================
            count_msg = Int32()

            count_msg.data = count

            self.count_pub.publish(
                count_msg
            )

            # =================================================
            # /yolo/detections
            # =================================================
            detection_msg = String()

            detection_msg.data = json.dumps(
                {
                    "count": count,
                    "detections": detections
                },
                ensure_ascii=False
            )

            self.detection_pub.publish(
                detection_msg
            )

            # =================================================
            # /yolo/centers
            # =================================================
            center_msg = String()

            center_msg.data = json.dumps(
                {
                    "count": len(centers),
                    "centers": centers
                },
                ensure_ascii=False
            )

            self.center_pub.publish(
                center_msg
            )

            # =================================================
            # Visualization
            # =================================================
            annotated = (
                result.plot().copy()
            )

            cv2.putText(
                annotated,
                f"Detections: {count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # =================================================
            # Center / Depth / Orientation 표시
            # =================================================
            for center in centers:

                cx = center["x"]
                cy = center["y"]

                depth = center["depth"]

                # ---------------------------------------------
                # Centroid 빨간 점
                # ---------------------------------------------
                cv2.circle(
                    annotated,
                    (cx, cy),
                    8,
                    (0, 0, 255),
                    -1
                )

                cv2.circle(
                    annotated,
                    (cx, cy),
                    11,
                    (255, 255, 255),
                    2
                )

                # ---------------------------------------------
                # Depth
                # ---------------------------------------------
                if depth is not None:

                    depth_text = (
                        f"D:{depth:.0f}"
                    )

                else:

                    depth_text = "D:N/A"

                # ---------------------------------------------
                # 방향축
                # ---------------------------------------------
                if (
                    "major_angle" in center
                    and "minor_angle" in center
                ):

                    major_angle = center[
                        "major_angle"
                    ]

                    minor_angle = center[
                        "minor_angle"
                    ]

                    gripper_angle = center[
                        "gripper_angle"
                    ]

                    # 긴 축
                    # 파란색
                    self.draw_axis(
                        annotated,
                        cx,
                        cy,
                        major_angle,
                        80,
                        (255, 0, 0),
                        3
                    )

                    # 짧은 축
                    # 그리퍼 closing 방향
                    # 노란색
                    self.draw_axis(
                        annotated,
                        cx,
                        cy,
                        minor_angle,
                        60,
                        (0, 255, 255),
                        3
                    )

                    angle_text = (
                        f"Grip:{gripper_angle:.1f}deg"
                    )

                else:

                    angle_text = "Grip:N/A"

                # ---------------------------------------------
                # 정보 출력
                # ---------------------------------------------
                cv2.putText(
                    annotated,
                    f"({cx},{cy}) {depth_text}",
                    (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    annotated,
                    angle_text,
                    (cx + 15, cy + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2
                )

            cv2.imshow(
                "D435i YOLO",
                annotated
            )

            rclpy.spin_once(
                self,
                timeout_sec=0
            )

            if (
                cv2.waitKey(1) & 0xFF
                == ord("q")
            ):

                break

        cv2.destroyAllWindows()


# =========================================================
# main
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = YoloTestNode()

    try:

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
