import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from kit_interfaces.msg import DetectionArray
from kit_interfaces.srv import GetComponentPose, InspectKit

PACKAGE_NAME = "kit_robot"

DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)

# Positions and offsets use millimeters.
DEFAULT_Z_OFFSET_MM = -35.0
MIN_TARGET_Z_MM = 2.0
DEFAULT_MAX_AGE_SEC = 1.0

WORKSPACE = {
    "x": (200.0, 800.0),
    "y": (-400.0, 400.0),
    "z": (0.0, 500.0),
}


def transform_to_base(camera_xyz, gripper2cam, robot_posx):
    x, y, z, rx, ry, rz = robot_posx

    base2gripper = np.eye(4, dtype=float)
    base2gripper[:3, :3] = Rotation.from_euler(
        "ZYZ",
        [rx, ry, rz],
        degrees=True,
    ).as_matrix()
    base2gripper[:3, 3] = [x, y, z]

    camera_point = np.append(np.asarray(camera_xyz, dtype=float), 1.0)
    return (base2gripper @ gripper2cam @ camera_point)[:3]


def mask_min_width_axis_angle(polygon_flat):
    if polygon_flat is None or len(polygon_flat) < 6:
        return None

    points = np.asarray(polygon_flat, dtype=np.float32).reshape(-1, 2)
    (_, _), (width, height), angle = cv2.minAreaRect(points)

    if width == 0 or height == 0:
        return None

    return float(angle if width < height else angle + 90.0)


def compute_target_pose(
    camera_xyz,
    masking_map,
    robot_posx,
    gripper2cam,
    z_offset,
    workspace,
):
    x, y, z = transform_to_base(camera_xyz, gripper2cam, robot_posx)
    z = max(z + z_offset, MIN_TARGET_Z_MM)

    if not (
        workspace["x"][0] <= x <= workspace["x"][1]
        and workspace["y"][0] <= y <= workspace["y"][1]
        and workspace["z"][0] <= z <= workspace["z"][1]
    ):
        return None, "out_of_workspace"

    rx, ry, rz = robot_posx[3:6]
    pixel_angle = mask_min_width_axis_angle(masking_map)
    if pixel_angle is not None:
        rz += pixel_angle

    return [
        float(x),
        float(y),
        float(z),
        float(rx),
        float(ry),
        float(rz),
    ], None


def select_candidates(objects, component, exclude_taken):
    excluded = set(exclude_taken or [])

    return [
        obj
        for obj in objects
        if obj.class_name == component
        and f"{obj.centroid_px[0]},{obj.centroid_px[1]}" not in excluded
    ]


def inspect_counts(objects, expected_classes, expected_counts):
    counts = Counter(obj.class_name for obj in objects)
    actual_counts = [counts.get(name, 0) for name in expected_classes]

    missing = [
        name
        for name, expected, actual in zip(
            expected_classes,
            expected_counts,
            actual_counts,
        )
        if actual < expected
    ]

    expected_set = set(expected_classes)
    unexpected = sorted(name for name in counts if name not in expected_set)
    ok = actual_counts == list(expected_counts) and not unexpected

    return ok, missing, unexpected, actual_counts


def load_gripper2cam(path):
    if not path.is_file():
        raise FileNotFoundError(f"Calibration file missing: {path}")

    transform = np.load(path)
    if transform.shape != (4, 4):
        raise ValueError(f"T_gripper2camera.npy must be 4x4, got {transform.shape}")
    if not np.all(np.isfinite(transform)):
        raise ValueError("T_gripper2camera.npy contains non-finite values")

    return transform


def load_grasp_z_offsets(path, logger):
    if not path.is_file():
        logger.warning(f"Grasp parameters missing: {path}; using default depth offset")
        return {}

    with path.open("r", encoding="utf-8") as file:
        params = json.load(file)

    return {
        name: values["z_offset"]
        for name, values in params.items()
        if "z_offset" in values
    }


class PositionEstimationNode(Node):
    def __init__(self):
        super().__init__("position_estimation_node")

        resource_dir = Path(get_package_share_directory(PACKAGE_NAME)) / "resource"
        logger = self.get_logger()

        self.gripper2cam = load_gripper2cam(
            resource_dir / "T_gripper2camera.npy",
        )
        self.z_offsets = load_grasp_z_offsets(
            resource_dir / "grasp_params.json",
            logger,
        )
        self.workspace = WORKSPACE
        self.default_z_offset = self.z_offsets.get(
            "_default",
            DEFAULT_Z_OFFSET_MM,
        )
        self.latest = None

        self.create_subscription(
            DetectionArray,
            "/detection/objects",
            self._on_detections,
            DETECTION_QOS,
        )
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

        logger.info("Position estimation node initialized")

    def _on_detections(self, message):
        self.latest = message

    def _age_sec(self):
        if self.latest is None:
            return float("inf")

        stamp = Time.from_msg(self.latest.header.stamp)
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def get_component_pose_callback(self, request, response):
        if self.latest is None:
            response.success = False
            response.error_code = "not_detected"
            return response

        max_age = (
            request.max_age_sec if request.max_age_sec > 0 else DEFAULT_MAX_AGE_SEC
        )
        age = self._age_sec()
        if age < 0.0 or age > max_age:
            response.success = False
            response.error_code = "stale"
            return response

        candidates = select_candidates(
            self.latest.objects,
            request.component,
            request.exclude_taken,
        )
        if not candidates:
            detected = any(
                obj.class_name == request.component for obj in self.latest.objects
            )
            response.success = False
            response.error_code = "no_candidate" if detected else "not_detected"
            return response

        robot_posx = list(request.robot_posx)
        z_offset = self.z_offsets.get(
            request.component,
            self.default_z_offset,
        )
        valid = []

        for obj in candidates:
            pose, error = compute_target_pose(
                list(obj.camera_xyz),
                list(obj.masking_map),
                robot_posx,
                self.gripper2cam,
                z_offset,
                self.workspace,
            )
            if error is None:
                valid.append((obj, pose))

        if not valid:
            response.success = False
            response.error_code = "out_of_workspace"
            response.source = candidates[0]
            return response

        source, pose = max(valid, key=lambda pair: pair[0].score)
        response.success = True
        response.target_pose = pose
        response.source = source
        response.error_code = ""
        return response

    def inspect_kit_callback(self, request, response):
        response.detection_age = self._age_sec()
        max_age = (
            request.max_age_sec if request.max_age_sec > 0 else DEFAULT_MAX_AGE_SEC
        )

        if (
            self.latest is None
            or response.detection_age < 0.0
            or response.detection_age > max_age
        ):
            response.ok = False
            response.missing = []
            response.unexpected = []
            response.actual_counts = [0] * len(request.expected_classes)
            return response

        ok, missing, unexpected, actual_counts = inspect_counts(
            self.latest.objects,
            list(request.expected_classes),
            list(request.expected_counts),
        )

        response.ok = ok
        response.missing = missing
        response.unexpected = unexpected
        response.actual_counts = actual_counts
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PositionEstimationNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()