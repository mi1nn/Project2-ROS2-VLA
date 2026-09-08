import json
import os
from collections import Counter

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from kit_interfaces.msg import DetectedObject, DetectionArray
from kit_interfaces.srv import GetComponentPose, InspectKit


# ---------------------------------------------------------------------------
# 기본 설정
# ---------------------------------------------------------------------------

PACKAGE_NAME = "kit_robot"

# 최신 Detection만 필요
DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)

# mm
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0
DEFAULT_MAX_AGE_SEC = 1.0

# mm
WORKSPACE = {
    "x": (200.0, 800.0),
    "y": (-400.0, 400.0),
    "z": (0.0, 500.0),
}


# ---------------------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------------------

def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    """
    Doosan posx 호환 형식:
        [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]

    orientation:
        ZYZ Euler degree

    반환:
        4x4 homogeneous transform
    """

    rotation = Rotation.from_euler(
        "ZYZ",
        [rx, ry, rz],
        degrees=True,
    ).as_matrix()

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = [x, y, z]

    return transform


def transform_to_base(
    camera_xyz,
    gripper2cam,
    robot_posx,
):
    """
    Camera coordinate -> Base coordinate

    eye-in-hand:

        T_base_camera
            =
        T_base_gripper
            @
        T_gripper_camera

    그 다음:

        P_base
            =
        T_base_camera
            @
        P_camera
    """

    x, y, z, rx, ry, rz = robot_posx

    base2gripper = get_robot_pose_matrix(
        x,
        y,
        z,
        rx,
        ry,
        rz,
    )

    base2cam = base2gripper @ gripper2cam

    camera_coord = np.append(
        np.asarray(camera_xyz, dtype=float),
        1.0,
    )

    base_coord = base2cam @ camera_coord

    return base_coord[:3]


# ---------------------------------------------------------------------------
# Mask angle
# ---------------------------------------------------------------------------

def mask_min_width_axis_angle(polygon_flat):
    """
    masking_map:
        [x1, y1, x2, y2, ...]

    최소 외접 사각형의 짧은 변 방향을 반환한다.
    """

    if polygon_flat is None:
        return None

    if len(polygon_flat) < 6:
        return None

    pts = np.asarray(
        polygon_flat,
        dtype=np.float32,
    ).reshape(-1, 2)

    (_, _), (w, h), angle = cv2.minAreaRect(pts)

    if w == 0 or h == 0:
        return None

    if w < h:
        return float(angle)

    return float(angle) + 90.0


def compute_rz(
    observe_rz,
    pixel_angle_deg,
):
    """
    관찰 자세 RZ에 이미지상의 물체 회전을 더한다.
    """

    return observe_rz + pixel_angle_deg


# ---------------------------------------------------------------------------
# 최종 target pose 계산
# ---------------------------------------------------------------------------

def compute_target_pose(
    camera_xyz,
    masking_map,
    robot_posx,
    gripper2cam,
    z_offset,
    workspace,
):
    """
    Camera XYZ -> Robot Base target pose

    반환:
        (
            [x, y, z, rx, ry, rz],
            None
        )

    실패:
        (
            None,
            "out_of_workspace"
        )
    """

    base_xyz = transform_to_base(
        camera_xyz,
        gripper2cam,
        robot_posx,
    )

    x, y, z = base_xyz

    # 파지 높이 보정
    z = max(
        z + z_offset,
        MIN_DEPTH,
    )

    wx = workspace["x"]
    wy = workspace["y"]
    wz = workspace["z"]

    if not (
        wx[0] <= x <= wx[1]
        and wy[0] <= y <= wy[1]
        and wz[0] <= z <= wz[1]
    ):
        return None, "out_of_workspace"

    rx = robot_posx[3]
    ry = robot_posx[4]
    observe_rz = robot_posx[5]

    pixel_angle = mask_min_width_axis_angle(
        masking_map
    )

    if pixel_angle is not None:
        rz = compute_rz(
            observe_rz,
            pixel_angle,
        )
    else:
        rz = observe_rz

    return [
        float(x),
        float(y),
        float(z),
        float(rx),
        float(ry),
        float(rz),
    ], None


# ---------------------------------------------------------------------------
# Detection 선택
# ---------------------------------------------------------------------------

def detection_key(obj):
    """
    현재 DetectedObject에 고유 ID가 없으므로
    centroid pixel을 임시 ID처럼 사용한다.
    """

    return (
        f"{obj.centroid_px[0]},"
        f"{obj.centroid_px[1]}"
    )


def select_candidates(
    objects,
    component,
    exclude_taken,
):
    """
    요청한 class와 일치하며
    이미 사용한 detection이 아닌 것만 반환.
    """

    exclude = set(
        exclude_taken or []
    )

    return [
        obj
        for obj in objects
        if (
            obj.class_name == component
            and detection_key(obj) not in exclude
        )
    ]


def inspect_counts(
    objects,
    expected_classes,
    expected_counts,
):
    """
    최종 kit 검사.
    """

    counts = Counter(
        obj.class_name
        for obj in objects
    )

    actual_counts = [
        counts.get(class_name, 0)
        for class_name in expected_classes
    ]

    missing = [
        class_name
        for class_name, expected, actual
        in zip(
            expected_classes,
            expected_counts,
            actual_counts,
        )
        if actual < expected
    ]

    expected_set = set(
        expected_classes
    )

    unexpected = sorted(
        class_name
        for class_name in counts
        if class_name not in expected_set
    )

    ok = (
        actual_counts == list(expected_counts)
        and not unexpected
    )

    return (
        ok,
        missing,
        unexpected,
        actual_counts,
    )


# ---------------------------------------------------------------------------
# Resource load
# ---------------------------------------------------------------------------

def load_gripper2cam(
    path,
    logger=None,
):
    """
    T_gripper2camera.npy 로드.

    없으면 기존 정책대로 identity를 사용하지만
    ERROR 로그를 반드시 출력한다.
    """

    if os.path.isfile(path):
        transform = np.load(path)

        if transform.shape != (4, 4):
            raise ValueError(
                "T_gripper2camera.npy must be 4x4, "
                f"got {transform.shape}"
            )

        if not np.all(
            np.isfinite(transform)
        ):
            raise ValueError(
                "T_gripper2camera.npy contains "
                "non-finite values"
            )

        return transform

    message = (
        "T_gripper2camera.npy 없음 "
        f"({path}) — 항등행렬 사용 중, "
        "캘리브레이션 전까지 좌표 신뢰 불가"
    )

    if logger:
        logger.error(message)
    else:
        print(message)

    return np.eye(4)


def load_grasp_z_offsets(
    path,
    logger=None,
):
    """
    grasp_params.json의 클래스별 z_offset 로드.
    """

    if not os.path.isfile(path):

        if logger:
            logger.warn(
                "grasp_params.json 없음 "
                f"({path}) — 전 품목 "
                "DEPTH_OFFSET 기본값 사용"
            )

        return {}

    with open(
        path,
        "r",
        encoding="utf-8",
    ) as file:

        raw = json.load(file)

    return {
        name: params["z_offset"]
        for name, params in raw.items()
        if "z_offset" in params
    }


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class PositionEstimationNode(Node):

    def __init__(self):

        super().__init__(
            "position_estimation_node"
        )

        # -------------------------------------------------------
        # Resource path
        # -------------------------------------------------------

        package_share = (
            get_package_share_directory(
                PACKAGE_NAME
            )
        )

        resource_dir = os.path.join(
            package_share,
            "resource",
        )

        gripper2cam_path = os.path.join(
            resource_dir,
            "T_gripper2camera.npy",
        )

        grasp_params_path = os.path.join(
            resource_dir,
            "grasp_params.json",
        )

        # -------------------------------------------------------
        # Hand-eye calibration
        # -------------------------------------------------------

        self.gripper2cam = (
            load_gripper2cam(
                gripper2cam_path,
                self.get_logger(),
            )
        )

        # 실제 어느 파일을 읽었는지 확인
        self.get_logger().warn(
            "[CALIB DEBUG] "
            f"T_gripper2camera path="
            f"{gripper2cam_path}"
        )

        self.get_logger().warn(
            "[CALIB DEBUG] "
            f"file_exists="
            f"{os.path.isfile(gripper2cam_path)}"
        )

        self.get_logger().warn(
            "[CALIB DEBUG] "
            "T_gripper2camera=\n"
            + np.array2string(
                self.gripper2cam,
                precision=6,
                suppress_small=True,
            )
        )

        rotation_matrix = (
            self.gripper2cam[:3, :3]
        )

        translation = (
            self.gripper2cam[:3, 3]
        )

        self.get_logger().warn(
            "[CALIB DEBUG] "
            f"translation={translation.tolist()}, "
            f"det(R)="
            f"{np.linalg.det(rotation_matrix):.6f}"
        )

        # -------------------------------------------------------
        # Grasp offset
        # -------------------------------------------------------

        self.z_offsets = (
            load_grasp_z_offsets(
                grasp_params_path,
                self.get_logger(),
            )
        )

        self.workspace = WORKSPACE

        # -------------------------------------------------------
        # Latest Detection
        # -------------------------------------------------------

        self.latest = None

        self.create_subscription(
            DetectionArray,
            "/detection/objects",
            self._on_detections,
            DETECTION_QOS,
        )

        # -------------------------------------------------------
        # Services
        # -------------------------------------------------------

        self.create_service(
            GetComponentPose,
            "/get_component_pose",
            self.get_component_pose_callback,
        )

        self.create_service(
            InspectKit,
            "/inspect_kit",
            self.inspect_kit_callback,
        )

        self.get_logger().info(
            "PositionEstimationNode initialized."
        )

    # ------------------------------------------------------------------
    # Detection callback
    # ------------------------------------------------------------------

    def _on_detections(
        self,
        msg,
    ):
        # 최신 것 하나만 보관
        self.latest = msg

    # ------------------------------------------------------------------
    # Detection age
    # ------------------------------------------------------------------

    def _age_sec(self):

        if self.latest is None:
            return float("inf")

        now = self.get_clock().now()

        stamp_time = Time.from_msg(
            self.latest.header.stamp
        )

        return (
            now - stamp_time
        ).nanoseconds / 1e9

    # ------------------------------------------------------------------
    # GetComponentPose
    # ------------------------------------------------------------------

    def get_component_pose_callback(
        self,
        request,
        response,
    ):

        # =======================================================
        # 1. Detection 자체가 아직 한번도 없을 경우
        # =======================================================

        if self.latest is None:

            self.get_logger().warn(
                "[DETECTION DEBUG] "
                f"request={request.component}, "
                "latest=None"
            )

            response.success = False
            response.error_code = "not_detected"

            return response

        # =======================================================
        # 2. Detection freshness
        # =======================================================

        age_sec = self._age_sec()

        max_age = (
            request.max_age_sec
            if request.max_age_sec > 0
            else DEFAULT_MAX_AGE_SEC
        )

        # 현재 Detection 내용을 요약해서 출력
        class_counts = Counter(
            obj.class_name
            for obj in self.latest.objects
        )

        detection_summary = []

        for obj in self.latest.objects:

            detection_summary.append({
                "class": obj.class_name,
                "score": round(
                    float(obj.score),
                    4,
                ),
                "camera_xyz": [
                    round(float(value), 3)
                    for value
                    in obj.camera_xyz
                ],
                "centroid": list(
                    obj.centroid_px
                ),
            })

        self.get_logger().warn(
            "[DETECTION DEBUG] "
            f"request={request.component}, "
            f"age={age_sec:.3f}s, "
            f"max_age={max_age:.3f}s, "
            f"class_counts={dict(class_counts)}, "
            f"objects={detection_summary}"
        )

        if age_sec > max_age:

            self.get_logger().warn(
                "[DETECTION DEBUG] "
                "result=stale"
            )

            response.success = False
            response.error_code = "stale"

            return response

        # =======================================================
        # 3. 요청 class candidate 검색
        # =======================================================

        candidates = select_candidates(
            self.latest.objects,
            request.component,
            request.exclude_taken,
        )

        if not candidates:

            # 주의:
            # 현재 select_candidates는 동일 클래스만 선택하기 때문에
            # 보통 여기에서 any_class=False라면 not_detected.
            any_class = any(
                obj.class_name
                == request.component
                for obj
                in self.latest.objects
            )

            error_code = (
                "no_candidate"
                if any_class
                else "not_detected"
            )

            self.get_logger().warn(
                "[DETECTION DEBUG] "
                f"candidate_count=0, "
                f"request='{request.component}', "
                f"available_classes="
                f"{list(class_counts.keys())}, "
                f"result={error_code}"
            )

            response.success = False
            response.error_code = error_code

            return response

        # =======================================================
        # 4. 좌표변환 준비
        # =======================================================

        z_offset = self.z_offsets.get(
            request.component,
            DEPTH_OFFSET,
        )

        robot_posx = list(
            request.robot_posx
        )

        self.get_logger().warn(
            "[COORD DEBUG] "
            f"candidate_count="
            f"{len(candidates)}, "
            f"robot_posx="
            f"{robot_posx}, "
            f"z_offset="
            f"{z_offset}"
        )

        # =======================================================
        # 5. Camera -> Base
        # =======================================================

        in_workspace = []

        for index, obj in enumerate(
            candidates,
            start=1,
        ):

            camera_xyz = list(
                obj.camera_xyz
            )

            # 원본 base 좌표
            base_xyz = transform_to_base(
                camera_xyz,
                self.gripper2cam,
                robot_posx,
            )

            # workspace 검사에 실제 사용되는 좌표
            corrected_xyz = (
                np.asarray(
                    base_xyz,
                    dtype=float,
                ).copy()
            )

            corrected_xyz[2] = max(
                corrected_xyz[2]
                + z_offset,
                MIN_DEPTH,
            )

            x = corrected_xyz[0]
            y = corrected_xyz[1]
            z = corrected_xyz[2]

            wx = self.workspace["x"]
            wy = self.workspace["y"]
            wz = self.workspace["z"]

            x_ok = (
                wx[0] <= x <= wx[1]
            )

            y_ok = (
                wy[0] <= y <= wy[1]
            )

            z_ok = (
                wz[0] <= z <= wz[1]
            )

            workspace_ok = (
                x_ok
                and y_ok
                and z_ok
            )

            self.get_logger().warn(
                "[COORD DEBUG] "
                f"candidate={index}/"
                f"{len(candidates)}, "
                f"class={obj.class_name}, "
                f"score={float(obj.score):.4f}, "
                f"camera_xyz="
                f"{camera_xyz}, "
                f"robot_posx="
                f"{robot_posx}"
            )

            self.get_logger().warn(
                "[COORD DEBUG] "
                f"base_xyz_raw="
                f"{base_xyz.tolist()}, "
                f"base_xyz_after_z_offset="
                f"{corrected_xyz.tolist()}"
            )

            self.get_logger().warn(
                "[WORKSPACE DEBUG] "
                f"x={x:.3f} "
                f"in {wx} -> {x_ok}, "
                f"y={y:.3f} "
                f"in {wy} -> {y_ok}, "
                f"z={z:.3f} "
                f"in {wz} -> {z_ok}, "
                f"overall={workspace_ok}"
            )

            # 기존 함수로 최종 pose 계산
            pose, err = compute_target_pose(
                camera_xyz,
                list(obj.masking_map),
                robot_posx,
                self.gripper2cam,
                z_offset,
                self.workspace,
            )

            self.get_logger().warn(
                "[COORD DEBUG] "
                f"target_pose={pose}, "
                f"error={err}"
            )

            if err is None:
                in_workspace.append(
                    (
                        obj,
                        pose,
                    )
                )

        # =======================================================
        # 6. 모든 candidate가 workspace 밖
        # =======================================================

        if not in_workspace:

            self.get_logger().warn(
                "[COORD DEBUG] "
                "모든 candidate가 "
                "workspace 밖입니다."
            )

            response.success = False
            response.error_code = (
                "out_of_workspace"
            )

            response.source = (
                candidates[0]
            )

            return response

        # =======================================================
        # 7. Workspace 내 candidate 중 confidence 최고값 선택
        # =======================================================

        obj, pose = max(
            in_workspace,
            key=lambda pair:
            pair[0].score,
        )

        self.get_logger().info(
            "[COORD DEBUG] "
            f"selected_class="
            f"{obj.class_name}, "
            f"score="
            f"{float(obj.score):.4f}, "
            f"target_pose="
            f"{pose}"
        )

        response.success = True
        response.target_pose = pose
        response.source = obj
        response.error_code = ""

        return response

    # ------------------------------------------------------------------
    # InspectKit
    # ------------------------------------------------------------------

    def inspect_kit_callback(
        self,
        request,
        response,
    ):

        response.detection_age = (
            self._age_sec()
            if self.latest is not None
            else float("inf")
        )

        max_age = (
            request.max_age_sec
            if request.max_age_sec > 0
            else DEFAULT_MAX_AGE_SEC
        )

        if (
            self.latest is None
            or response.detection_age
            > max_age
        ):

            response.ok = False
            response.missing = []
            response.unexpected = []

            response.actual_counts = (
                [0]
                * len(
                    request.expected_classes
                )
            )

            return response

        (
            ok,
            missing,
            unexpected,
            actual_counts,
        ) = inspect_counts(
            self.latest.objects,
            list(
                request.expected_classes
            ),
            list(
                request.expected_counts
            ),
        )

        response.ok = ok
        response.missing = missing
        response.unexpected = unexpected
        response.actual_counts = actual_counts

        return response


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        PositionEstimationNode()
    )

    try:
        rclpy.spin(node)

    finally:

        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------

def _demo():

    # -------------------------------------------------------
    # Identity hand-eye
    # -------------------------------------------------------

    transform = np.eye(4)

    assert np.allclose(
        transform_to_base(
            [100, 0, 500],
            transform,
            [0, 0, 0, 0, 0, 0],
        ),
        [100, 0, 500],
    )

    # -------------------------------------------------------
    # Base X translation
    # -------------------------------------------------------

    assert np.allclose(
        transform_to_base(
            [100, 0, 500],
            transform,
            [200, 0, 0, 0, 0, 0],
        ),
        [300, 0, 500],
    )

    # -------------------------------------------------------
    # Mask angle
    # -------------------------------------------------------

    assert (
        mask_min_width_axis_angle(
            [1, 2, 3]
        )
        is None
    )

    assert (
        mask_min_width_axis_angle(
            None
        )
        is None
    )

    angle = (
        mask_min_width_axis_angle(
            [
                0, 0,
                100, 0,
                100, 20,
                0, 20,
            ]
        )
    )

    assert (
        angle is not None
        and np.isfinite(angle)
    )

    # -------------------------------------------------------
    # Workspace fail
    # -------------------------------------------------------

    workspace = WORKSPACE

    robot_posx = [
        0,
        0,
        0,
        0,
        0,
        0,
    ]

    pose, err = compute_target_pose(
        [100, 0, 500],
        None,
        robot_posx,
        transform,
        -35.0,
        workspace,
    )

    assert (
        pose is None
        and err
        == "out_of_workspace"
    ), (
        pose,
        err,
    )

    # -------------------------------------------------------
    # Workspace success
    # -------------------------------------------------------

    pose, err = compute_target_pose(
        [300, 0, 500],
        None,
        robot_posx,
        transform,
        -35.0,
        workspace,
    )

    assert (
        err is None
        and pose
        == [
            300.0,
            0.0,
            465.0,
            0.0,
            0.0,
            0.0,
        ]
    ), (
        pose,
        err,
    )

    # -------------------------------------------------------
    # Candidate select
    # -------------------------------------------------------

    class _Obj:

        def __init__(
            self,
            class_name,
            centroid_px,
            score,
        ):

            self.class_name = (
                class_name
            )

            self.centroid_px = (
                centroid_px
            )

            self.score = score

    objs = [
        _Obj(
            "cup_ramen",
            [10, 10],
            0.9,
        ),
        _Obj(
            "cup_ramen",
            [50, 50],
            0.8,
        ),
        _Obj(
            "mask",
            [30, 30],
            0.95,
        ),
    ]

    assert (
        len(
            select_candidates(
                objs,
                "cup_ramen",
                [],
            )
        )
        == 2
    )

    remaining = (
        select_candidates(
            objs,
            "cup_ramen",
            ["10,10"],
        )
    )

    assert (
        len(remaining) == 1
        and remaining[0].centroid_px
        == [50, 50]
    )

    # -------------------------------------------------------
    # Inspection
    # -------------------------------------------------------

    (
        ok,
        missing,
        unexpected,
        actual,
    ) = inspect_counts(
        objs,
        [
            "cup_ramen",
            "mask",
        ],
        [
            2,
            1,
        ],
    )

    assert (
        ok
        and missing == []
        and unexpected == []
        and actual == [2, 1]
    ), (
        ok,
        missing,
        unexpected,
        actual,
    )

    (
        ok,
        missing,
        unexpected,
        actual,
    ) = inspect_counts(
        objs,
        ["cup_ramen"],
        [3],
    )

    assert (
        not ok
        and missing
        == ["cup_ramen"]
        and unexpected
        == ["mask"]
    ), (
        ok,
        missing,
        unexpected,
        actual,
    )

    print("ok")


if __name__ == "__main__":
    _demo()
