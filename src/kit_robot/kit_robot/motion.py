import json
import math
import os
import threading
import time
import warnings

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from kit_interfaces.msg import DetectionArray
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .onrobot import RG


def merge_octomap_acm(entry_names, matrix, allowed_links):
    """Return (names, matrix) with "<octomap>" allowed against allowed_links.

    Pure list surgery so it can be checked without a running move_group. The
    input must be the *current* ACM: a PlanningScene diff carrying a non-empty
    ACM replaces the whole matrix, so anything dropped here (the SRDF
    self-collision pairs) stops being ignored and planning dies on self-hits.
    """
    names = list(entry_names)
    matrix = [list(row) for row in matrix]

    # "<octomap>" already exists if move_group kept running across a restart of
    # this node; appending a name twice would leave a stale, unused row.
    for name in ["<octomap>"] + list(allowed_links):
        if name in names:
            continue
        names.append(name)
        for row in matrix:
            row.append(False)
        matrix.append([False] * len(names))

    octomap_index = names.index("<octomap>")
    for name in allowed_links:
        index = names.index(name)
        matrix[octomap_index][index] = True
        matrix[index][octomap_index] = True

    return names, matrix


def _mask_cloud_polygon(
    xyz, polygons_px, intrinsics, width, height, padding_px=0
):
    """Return a bool array: True for points to KEEP (outside the YOLO masks).

    ``xyz`` is Nx3 in the camera optical frame (any linear unit — the
    projection is a ratio, so meters vs mm doesn't matter). ``polygons_px`` is
    a list of DetectedObject.masking_map, each a flat [x0, y0, x1, y1, ...]
    pixel polygon in the color image ``intrinsics`` belongs to — one class can
    have several instances in frame at once, and we exclude the union of all
    of them (picking just one, e.g. "the last one in the message", risks
    excluding the wrong instance and leaving the real pick target un-excluded).
    Points that can't be projected (non-finite/non-positive depth, or landing
    outside the image) are kept — better to occasionally leak one unmasked
    point than to silently carve out unrelated geometry.

    Pure buffer/geometry math so it can be checked without rclpy/sensor_msgs.
    """
    n_points = xyz.shape[0]
    if n_points == 0:
        return np.zeros(0, dtype=bool)

    mask_image = np.zeros((height, width), dtype=np.uint8)
    contours = [
        np.round(np.asarray(polygon_px, dtype=np.float32).reshape(-1, 2)).astype(
            np.int32
        )
        for polygon_px in polygons_px
    ]
    cv2.fillPoly(mask_image, contours, 1)
    if padding_px:
        padding_px = int(padding_px)
        if padding_px < 0:
            raise ValueError("padding_px must be non-negative")
        diameter = 2 * padding_px + 1
        mask_image = cv2.dilate(
            mask_image,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter)),
        )

    z = xyz[:, 2]
    projectable = np.isfinite(z) & (z > 0.0)

    u = np.zeros(n_points, dtype=np.int64)
    v = np.zeros(n_points, dtype=np.int64)
    u[projectable] = np.round(
        xyz[projectable, 0] / z[projectable] * intrinsics["fx"] + intrinsics["ppx"]
    ).astype(np.int64)
    v[projectable] = np.round(
        xyz[projectable, 1] / z[projectable] * intrinsics["fy"] + intrinsics["ppy"]
    ).astype(np.int64)

    in_bounds = projectable & (u >= 0) & (u < width) & (v >= 0) & (v < height)
    excluded = np.zeros(n_points, dtype=bool)
    excluded[in_bounds] = mask_image[v[in_bounds], u[in_bounds]].astype(bool)
    return ~excluded


def _remove_outlier_points(xyz, voxel_size=0.01, min_neighbors=4):
    """Return a bool keep-mask that drops sparse "flying pixel" noise.

    A real surface (table, tray, object) returns a locally dense cluster of
    points; a depth-sensor flying pixel — the classic stray return at an
    object edge, floating between foreground and background — sits alone in
    its neighborhood. Bucket points into a coarse voxel grid and keep only
    points whose bucket holds at least ``min_neighbors`` points total
    (including itself); this both flattens edge scatter onto whichever real
    surface it's nearest to and drops isolated outliers outright. NaN/Inf/
    non-positive-depth points are dropped unconditionally.

    ``voxel_size`` should be a few mm to a cm or so — coarser than
    ``octomap_resolution`` (this filters points, not the octree itself) but
    fine enough not to merge separate objects into one bucket.

    Pure buffer/geometry math so it can be checked without rclpy/sensor_msgs.
    """
    n_points = xyz.shape[0]
    keep = np.zeros(n_points, dtype=bool)
    if n_points == 0:
        return keep

    finite = np.all(np.isfinite(xyz), axis=1) & (xyz[:, 2] > 0.0)
    if not np.any(finite):
        return keep

    # Hash each voxel to one int64 key instead of np.unique(..., axis=0),
    # which is far slower at cloud-sized point counts. OFFSET keeps indices
    # non-negative and comfortably covers D435's few-meter range at any
    # sane voxel_size.
    OFFSET = 1 << 16
    idx = np.floor(xyz[finite] / voxel_size).astype(np.int64) + OFFSET
    keys = (idx[:, 0] * OFFSET + idx[:, 1]) * OFFSET + idx[:, 2]
    _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    keep[finite] = counts[inverse] >= min_neighbors
    return keep


def _flatten_floor_points(xyz, band_m=0.015, floor_percentile=5.0):
    """Snap near-floor height wobble flat onto one reference height.

    ``xyz`` must already be in a frame where z is "up" (base_frame) — the
    eye-in-hand camera frame tilts with the arm, so a flat floor is not a
    constant-z plane there. Grazing-angle depth quantization draws the floor
    as a staircase of locally-coherent "shelves" — each shelf is internally
    dense, so ``_remove_outlier_points`` (which only drops isolated points)
    doesn't touch it. This instead assumes the floor/tray is the dominant,
    lowest surface in view: the ``floor_percentile``-th lowest finite z is
    taken as the true floor height, and any point within ``band_m`` of it is
    pulled exactly onto that height. Points farther above it (real objects,
    tray walls) are left untouched, so ``band_m`` must stay smaller than the
    shortest real object that still needs to register as an obstacle.

    Pure numpy so it can be checked without rclpy.
    """
    out = xyz.copy()
    z = out[:, 2]
    finite = np.isfinite(z)
    if not np.any(finite):
        return out
    floor_z = np.percentile(z[finite], floor_percentile)
    near_floor = finite & (np.abs(z - floor_z) <= band_m)
    out[near_floor, 2] = floor_z
    return out


class Motion:
    """MoveIt2 based motion backend.

    Public methods intentionally keep the same contract as the previous DSR_ROBOT2
    implementation so Controller and PositionEstimationNode do not need to know which
    robot motion backend is in use.

    External pose convention kept for compatibility:
      [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    where orientation is ZYZ Euler, matching the previous Doosan posx convention used
    by position_estimation.py and the hand-eye calibration data.
    """

    def __init__(self, node):
        if node is None:
            raise ValueError("ROS 2 node is required.")

        self.node = node
        self.logger = node.get_logger()

        config_path = os.path.join(
            get_package_share_directory("kit_robot"), "config", "motion.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)["motion"]

        self.positions = config["positions"]
        self.place_config = config["place"]
        self.place_slots = self.place_config["slots"]
        self.moveit_config = config.get("moveit", {})

        grasp_params_path = os.path.join(
            get_package_share_directory("kit_robot"),
            "resource",
            "grasp_params.json",
        )
        with open(grasp_params_path, "r", encoding="utf-8") as file:
            self.grasp_params = json.load(file)

        # MoveIt / TF configuration. These defaults match the standard Doosan M0609
        # MoveIt model. If the custom RG2 SRDF uses another tip, change eef_link only.
        self.group_name = self.moveit_config.get("planning_group", "manipulator")
        self.base_frame = self.moveit_config.get("base_frame", "base_link")
        self.eef_link = self.moveit_config.get("eef_link", "tool0")
        self.joint_names = list(
            self.moveit_config.get(
                "joint_names",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            )
        )
        if len(self.joint_names) != 6:
            raise ValueError("moveit.joint_names must contain exactly 6 joints.")

        self.move_group_action_name = self.moveit_config.get(
            "move_group_action", "/dsr01/move_action"
        )
        self.execute_action_name = self.moveit_config.get(
            "execute_trajectory_action", "/execute_trajectory"
        )
        self.cartesian_service_name = self.moveit_config.get(
            "cartesian_path_service", "/compute_cartesian_path"
        )
        self.apply_planning_scene_service_name = self.moveit_config.get(
            "apply_planning_scene_service", "/dsr01/apply_planning_scene"
        )

        # Fixed keepout box. Bounds are configured in centimeters and converted
        # to meters only when messages are sent to MoveIt/RViz. Other Motion pose
        # APIs remain millimeter-based for compatibility with the existing project.
        self.keepout_config = self.moveit_config.get("default_keepout_box", {})
        self.keepout_enabled = bool(self.keepout_config.get("enabled", True))
        self.keepout_object_id = str(
            self.keepout_config.get("id", "base_keepout_box")
        )
        self.keepout_min_cm = list(
            self.keepout_config.get("min_cm", [-200.0, 40.0, -100.0])
        )
        self.keepout_max_cm = list(
            self.keepout_config.get("max_cm", [200.0, 100.0, 600.0])
        )
        self.keepout_marker_topic = str(
            self.keepout_config.get(
                "marker_topic", "/kit_robot/default_keepout_box_marker"
            )
        )

        # Octomap gate. cloud_out must match point_cloud_topic in the MoveIt
        # config's sensors_3d.yaml, otherwise the octomap stays empty silently.
        self.octomap_config = self.moveit_config.get("octomap", {})
        self.octomap_enabled = bool(self.octomap_config.get("enabled", True))
        self.octomap_cloud_in = str(
            self.octomap_config.get("cloud_in", "/camera/depth/color/points")
        )
        self.octomap_cloud_out = str(
            self.octomap_config.get("cloud_out", "/kit/octomap_cloud")
        )
        self.octomap_clear_service = str(
            self.octomap_config.get("clear_service", "/dsr01/clear_octomap")
        )
        self.octomap_get_scene_service = str(
            self.octomap_config.get(
                "get_planning_scene_service", "/dsr01/get_planning_scene"
            )
        )
        # 옥토맵으로 들어가기 전 클라우드 디노이즈 파라미터. _remove_outlier_points 참고.
        self.octomap_denoise_voxel_m = float(
            self.octomap_config.get("denoise_voxel_size_m", 0.01)
        )
        self.octomap_denoise_min_neighbors = int(
            self.octomap_config.get("denoise_min_neighbors", 4)
        )
        # 그레이징 앵글 계단 노이즈 펴기. _flatten_floor_points 참고 — band_m
        # 은 실제로 장애물로 봐야 할 가장 낮은 물체보다 작아야 한다(지금
        # 5cm 짜리 물체도 집어야 하므로 그보다 한참 작게 잡는다).
        self.octomap_flatten_floor_enabled = bool(
            self.octomap_config.get("flatten_floor_enabled", True)
        )
        self.octomap_flatten_band_m = float(
            self.octomap_config.get("flatten_floor_band_m", 0.015)
        )
        self.octomap_flatten_percentile = float(
            self.octomap_config.get("flatten_floor_percentile", 5.0)
        )
        self.octomap_exclusion_mask_padding_px = int(
            self.octomap_config.get("exclusion_mask_padding_px", 15)
        )
        if self.octomap_exclusion_mask_padding_px < 0:
            raise ValueError("octomap.exclusion_mask_padding_px must be >= 0")
        # Links allowed to collide with octomap voxels: the gripper and what is
        # bolted to it. The target object and the table are voxels too, so
        # without this no grasp is ever plannable. Upper arm links stay out.
        self.octomap_allowed_links = [
            str(name)
            for name in self.octomap_config.get(
                "allowed_collision_links",
                ["link_6", "tool0", "rg2_base_link"],
            )
        ]
        # 집을 물체 자체가 옥토맵에 박혀도, 위 ACM 면제는 그리퍼 링크까지만
        # 풀어준다 — 손목/팔뚝 접근 경로는 여전히 막힌다. 부분 삭제 API가
        # 없어서 한 번 voxel 로 박히면 못 빼므로, clear 대신 애초에 그 물체의
        # YOLO 마스크에 해당하는 depth 포인트를 옥토맵으로 보내기 전에
        # 걸러낸다 (_filter_octomap_cloud / set_octomap_exclusion_component).

        self.server_timeout = float(
            self.moveit_config.get("server_ready_timeout_sec", 15.0)
        )
        self.motion_timeout = float(
            self.moveit_config.get("motion_timeout_sec", 60.0)
        )
        self.tf_timeout = float(self.moveit_config.get("tf_timeout_sec", 2.0))
        self.planning_time = float(
            self.moveit_config.get("planning_time_sec", 5.0)
        )
        self.planning_attempts = int(
            self.moveit_config.get("planning_attempts", 5)
        )
        self.pipeline_id = str(self.moveit_config.get("pipeline_id", ""))
        self.planner_id = str(self.moveit_config.get("planner_id", ""))

        self.joint_tolerance_rad = math.radians(
            float(self.moveit_config.get("joint_tolerance_deg", 0.5))
        )
        self.default_joint_velocity_scale = self._clamp_scale(
            self.moveit_config.get("default_joint_velocity_scale", 0.15)
        )
        self.default_joint_acceleration_scale = self._clamp_scale(
            self.moveit_config.get("default_joint_acceleration_scale", 0.15)
        )

        self.cartesian_max_step_m = float(
            self.moveit_config.get("cartesian_max_step_m", 0.005)
        )
        self.cartesian_min_fraction = float(
            self.moveit_config.get("cartesian_min_fraction", 0.999)
        )
        self.cartesian_jump_threshold = float(
            self.moveit_config.get("cartesian_jump_threshold", 0.0)
        )
        self.cartesian_prismatic_jump_threshold = float(
            self.moveit_config.get("cartesian_prismatic_jump_threshold", 0.0)
        )
        self.cartesian_revolute_jump_threshold = float(
            self.moveit_config.get("cartesian_revolute_jump_threshold", 0.0)
        )
        self.cartesian_acc_reference_mm_s2 = float(
            self.moveit_config.get("cartesian_acc_reference_mm_s2", 1000.0)
        )
        if self.cartesian_max_step_m <= 0.0:
            raise ValueError("moveit.cartesian_max_step_m must be > 0.")
        if not 0.0 < self.cartesian_min_fraction <= 1.0:
            raise ValueError("moveit.cartesian_min_fraction must be in (0, 1].")
        if self.cartesian_acc_reference_mm_s2 <= 0.0:
            raise ValueError("moveit.cartesian_acc_reference_mm_s2 must be > 0.")

        # A dedicated node/executor lets Motion synchronously wait for MoveIt actions
        # without blocking Controller's state-machine node or nesting rclpy.spin calls.
        self._moveit_node = rclpy.create_node("kit_moveit_interface")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._moveit_node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="kit_moveit_executor",
            daemon=True,
        )
        self._spin_thread.start()

        self._move_group_client = ActionClient(
            self._moveit_node, MoveGroup, self.move_group_action_name
        )
        self._execute_client = ActionClient(
            self._moveit_node, ExecuteTrajectory, self.execute_action_name
        )
        self._cartesian_client = self._moveit_node.create_client(
            GetCartesianPath, self.cartesian_service_name
        )
        self._apply_planning_scene_client = self._moveit_node.create_client(
            ApplyPlanningScene, self.apply_planning_scene_service_name
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._keepout_marker_pub = self._moveit_node.create_publisher(
            Marker, self.keepout_marker_topic, marker_qos
        )

        # Open by default so a run that never reaches the observation pose (or a
        # bringup without Controller) still shows a map in RViz.
        self._octomap_mapping = self.octomap_enabled
        # True 이면 지도가 "확정"됐다는 뜻: 게이트는 다시 열리지 않고
        # clear_octomap() 도 먹지 않는다. Controller 가 작업당 딱 두 번
        # (키팅 트레이 1회 + 홈 자세 장시간 1회) 쌓고 나서 freeze_octomap()
        # 으로 잠근다 — 이후 픽·플레이스·검사 전 구간이 이 지도 하나만 본다.
        # 새 작업 시작 시 move_to_inspection_pose(clear_before=True) 가 푼다.
        self._octomap_frozen = False
        # 지금 옥토맵에서 빼고 있는 컴포넌트 이름(클래스명) 또는 None.
        # Controller 가 move_to_observation_pose() 호출 *전에* 걸어야 한다 —
        # 부분 삭제 API가 없어서, 게이트가 열리고 들어오는 첫 프레임부터 이미
        # 이 물체의 voxel 이 박히기 시작한다.
        self._octomap_exclusion_component = None
        self._octomap_exclusion_min_stamp_ns = None
        # class_name -> DetectedObject.masking_map (최신 1개만 유지).
        self._latest_detection_masks = {}
        self._latest_detection_stamp_ns = None
        # {"fx", "fy", "ppx", "ppy"} 또는 None (아직 camera_info 못 받음).
        self._camera_intrinsics = None
        self._camera_width = None
        self._camera_height = None
        self._clear_octomap_client = None
        self._get_planning_scene_client = None
        if self.octomap_enabled:
            self._clear_octomap_client = self._moveit_node.create_client(
                Empty, self.octomap_clear_service
            )
            self._get_planning_scene_client = self._moveit_node.create_client(
                GetPlanningScene, self.octomap_get_scene_service
            )
            self._octomap_cloud_pub = self._moveit_node.create_publisher(
                PointCloud2, self.octomap_cloud_out, qos_profile_sensor_data
            )
            # ponytail: Python relay of a 848x480 cloud. Only open while the arm
            # is parked, and the updater throttles to max_update_rate anyway. If
            # CPU matters, point sensors_3d.yaml at the camera topic and gate the
            # driver instead.
            self._moveit_node.create_subscription(
                PointCloud2,
                self.octomap_cloud_in,
                self._octomap_cloud_callback,
                qos_profile_sensor_data,
            )
            # pick 위치에서 집을 물체의 YOLO 마스크를 옥토맵에서 걸러내려면
            # 픽셀 폴리곤(masking_map)과 그걸 찍은 카메라의 intrinsics 가
            # 둘 다 필요하다 — kit_vision object_detection.py 와 독립적으로,
            # 여기서 바로 구독한다.
            detection_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, depth=1
            )
            self._moveit_node.create_subscription(
                DetectionArray,
                "/detection/objects",
                self._detection_callback,
                detection_qos,
            )
            self._moveit_node.create_subscription(
                CameraInfo,
                "/camera/color/camera_info",
                self._camera_info_callback,
                qos_profile_sensor_data,
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._moveit_node, spin_thread=False
        )

        self.rg = RG("rg2", "192.168.1.1", 502)

        self._wait_for_moveit_servers()
        self._add_default_keepout_box()
        self._allow_octomap_collisions()
        self.logger.info(
            "MoveIt2 Motion initialized: "
            f"group={self.group_name}, base={self.base_frame}, eef={self.eef_link}"
        )

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp_scale(value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("MoveIt scaling factor must be finite.")
        return min(1.0, max(0.001, value))

    def _wait_for_moveit_servers(self):
        if not self._move_group_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"MoveGroup action unavailable: {self.move_group_action_name}"
            )
        if not self._execute_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"ExecuteTrajectory action unavailable: {self.execute_action_name}"
            )
        if not self._cartesian_client.wait_for_service(timeout_sec=self.server_timeout):
            raise RuntimeError(
                f"GetCartesianPath service unavailable: {self.cartesian_service_name}"
            )
        # Needed by both the keepout box and the octomap ACM merge.
        needs_planning_scene = self.keepout_enabled or self.octomap_enabled
        if needs_planning_scene and not self._apply_planning_scene_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            raise RuntimeError(
                "ApplyPlanningScene service unavailable: "
                f"{self.apply_planning_scene_service_name}"
            )

        # Missing octomap services mean move_group has no occupancy map monitor.
        # Without GetPlanningScene the ACM cannot be merged, and an un-excused
        # octomap makes every grasp unplannable — so drop the map entirely
        # instead of dying or bricking the run.
        for client, name in (
            (self._clear_octomap_client, self.octomap_clear_service),
            (self._get_planning_scene_client, self.octomap_get_scene_service),
        ):
            if not self.octomap_enabled:
                break
            if client.wait_for_service(timeout_sec=self.server_timeout):
                continue
            self.logger.warning(
                f"Octomap service unavailable: {name} — octomap disabled. "
                "Obstacle avoidance falls back to the keepout box only."
            )
            self.octomap_enabled = False
            self._octomap_mapping = False

    def _wait_future(self, future, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{description} timed out after {timeout_sec:.1f}s")
            time.sleep(0.01)

        if not rclpy.ok() and not future.done():
            raise RuntimeError(f"ROS shutdown while waiting for {description}")
        if future.cancelled():
            raise RuntimeError(f"{description} was cancelled")

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{description} failed: {exception}") from exception
        return future.result()

    @staticmethod
    def _error_name(code):
        # Keep the numeric value because generated Python messages do not expose
        # a guaranteed enum-to-string helper across MoveIt releases.
        return f"MoveItErrorCodes({code})"

    @staticmethod
    def _pose6_to_ros_pose(pose6):
        pose = list(pose6)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")
        if not all(math.isfinite(float(v)) for v in pose):
            raise ValueError("target_pose contains non-finite values")

        msg = Pose()
        msg.position.x = float(pose[0]) / 1000.0
        msg.position.y = float(pose[1]) / 1000.0
        msg.position.z = float(pose[2]) / 1000.0

        quat = Rotation.from_euler(
            "ZYZ", [float(pose[3]), float(pose[4]), float(pose[5])], degrees=True
        ).as_quat()  # scipy order: x, y, z, w
        msg.orientation.x = float(quat[0])
        msg.orientation.y = float(quat[1])
        msg.orientation.z = float(quat[2])
        msg.orientation.w = float(quat[3])
        return msg

    def _joint_scales(self, config):
        vel_scale = config.get(
            "joint_velocity_scale", self.default_joint_velocity_scale
        )
        acc_scale = config.get(
            "joint_acceleration_scale", self.default_joint_acceleration_scale
        )
        return self._clamp_scale(vel_scale), self._clamp_scale(acc_scale)

    @staticmethod
    def _validate_keepout_bounds(min_cm, max_cm):
        if len(min_cm) != 3 or len(max_cm) != 3:
            raise ValueError(
                "default_keepout_box min_cm/max_cm must contain [x, y, z]"
            )

        min_values = [float(v) for v in min_cm]
        max_values = [float(v) for v in max_cm]

        if not all(math.isfinite(v) for v in min_values + max_values):
            raise ValueError("default_keepout_box contains non-finite values")

        for axis, low, high in zip("xyz", min_values, max_values):
            if high <= low:
                raise ValueError(
                    f"default_keepout_box max_{axis} must be greater than min_{axis}"
                )

        return min_values, max_values

    def _keepout_geometry_m(self):
        min_cm, max_cm = self._validate_keepout_bounds(
            self.keepout_min_cm, self.keepout_max_cm
        )

        # cm -> m
        center_m = [
            (low + high) * 0.5 / 100.0
            for low, high in zip(min_cm, max_cm)
        ]
        size_m = [
            (high - low) / 100.0
            for low, high in zip(min_cm, max_cm)
        ]
        return min_cm, max_cm, center_m, size_m

    def _publish_keepout_marker(self, center_m, size_m):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self._moveit_node.get_clock().now().to_msg()
        marker.ns = "kit_robot_keepout"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = center_m[0]
        marker.pose.position.y = center_m[1]
        marker.pose.position.z = center_m[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = size_m[0]
        marker.scale.y = size_m[1]
        marker.scale.z = size_m[2]

        # Semi-transparent red keepout visualization in RViz.
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.30

        self._keepout_marker_pub.publish(marker)

    def _add_default_keepout_box(self):
        if not self.keepout_enabled:
            self.logger.info("Default keepout box disabled")
            return False

        min_cm, max_cm, center_m, size_m = self._keepout_geometry_m()

        collision = CollisionObject()
        collision.header.frame_id = self.base_frame
        collision.header.stamp = self._moveit_node.get_clock().now().to_msg()
        collision.id = self.keepout_object_id
        collision.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size_m)

        box_pose = Pose()
        box_pose.position.x = center_m[0]
        box_pose.position.y = center_m[1]
        box_pose.position.z = center_m[2]
        box_pose.orientation.w = 1.0

        collision.primitives = [box]
        collision.primitive_poses = [box_pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_planning_scene_client.call_async(request)
        response = self._wait_future(
            future,
            self.server_timeout,
            "ApplyPlanningScene(default_keepout_box)",
        )

        if response is None or not response.success:
            raise RuntimeError("Failed to add default keepout box")

        self._publish_keepout_marker(center_m, size_m)

        center_cm = [value * 100.0 for value in center_m]
        size_cm = [value * 100.0 for value in size_m]
        self.logger.info(
            "Default keepout box added: "
            f"id={self.keepout_object_id}, "
            f"frame={self.base_frame}, "
            f"min_cm={min_cm}, max_cm={max_cm}, "
            f"center_cm={[round(v, 3) for v in center_cm]}, "
            f"size_cm={[round(v, 3) for v in size_cm]}, "
            f"marker_topic={self.keepout_marker_topic}"
        )
        return True

    # ------------------------------------------------------------------
    # Octomap (MoveIt occupancy map monitor)
    # ------------------------------------------------------------------

    def _octomap_cloud_callback(self, msg):
        if not self._octomap_mapping:
            return
        if (
            self._octomap_exclusion_component is not None
            and not self._has_fresh_exclusion_mask()
        ):
            self.logger.warn(
                "옥토맵 중계 보류: 파지 대상의 새 YOLO 마스크가 아직 없음 "
                f"(component={self._octomap_exclusion_component})",
                throttle_duration_sec=2.0,
            )
            return
        if (
            self._octomap_exclusion_component is not None
            and self._camera_intrinsics is None
        ):
            self.logger.warn(
                "옥토맵 중계 보류: 파지 대상 마스크를 투영할 camera_info가 없음",
                throttle_duration_sec=2.0,
            )
            return
        self._octomap_cloud_pub.publish(self._filter_octomap_cloud(msg))

    def _detection_callback(self, msg):
        # class_name 하나에 인스턴스가 여러 개일 수 있다(같은 품목 2개 이상).
        # 마지막 걸로 덮어쓰면 실제로 집으려는 인스턴스가 아닌 다른 인스턴스의
        # 마스크만 남아 정작 걸러야 할 물체는 그대로 새는 사고가 난다 — 이번
        # 메시지에 잡힌 같은 클래스는 전부 모아 합집합으로 제외한다.
        masks = {}
        for obj in msg.objects:
            masks.setdefault(obj.class_name, []).append(list(obj.masking_map))
        self._latest_detection_masks = masks
        stamp = msg.header.stamp
        self._latest_detection_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

    def _has_fresh_exclusion_mask(self):
        component = self._octomap_exclusion_component
        if component is None:
            return True
        if not self._latest_detection_masks.get(component):
            return False
        minimum = getattr(self, "_octomap_exclusion_min_stamp_ns", None)
        stamp = getattr(self, "_latest_detection_stamp_ns", None)
        return minimum is None or (stamp is not None and stamp >= minimum)

    def _camera_info_callback(self, msg):
        self._camera_intrinsics = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "ppx": float(msg.k[2]),
            "ppy": float(msg.k[5]),
        }
        self._camera_width = msg.width
        self._camera_height = msg.height

    def set_octomap_exclusion_component(self, component_name):
        """Start dropping `component_name`'s YOLO mask from every relayed
        cloud frame, from now on.

        Call this *before* move_to_observation_pose(), not after — there is
        no partial-octree erase, so any frame relayed before this runs can
        still bake the object's voxels in permanently.
        """
        if not self.octomap_enabled:
            return
        self._octomap_exclusion_component = component_name
        self._octomap_exclusion_min_stamp_ns = self._moveit_node.get_clock().now().nanoseconds

    def clear_octomap_exclusion(self):
        self._octomap_exclusion_component = None
        self._octomap_exclusion_min_stamp_ns = None

    def _flatten_floor_in_sensor_frame(self, xyz, header):
        """Round-trip ``xyz`` through base_frame to flatten floor wobble,
        then hand it back in the cloud's own (sensor) frame.

        ``_flatten_floor_points`` needs z to mean "up", which only holds in
        base_frame — the eye-in-hand camera frame tilts with every joint
        move. TF lookup failure just skips flattening for this one frame
        (denoise/exclusion below still run) rather than dropping the cloud.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                header.frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:
            self.logger.warn(
                f"옥토맵 평면 펴기 TF 조회 실패, 이번 프레임은 건너뜀: {error}",
                throttle_duration_sec=5.0,
            )
            return xyz

        t = transform.transform.translation
        q = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])

        base_xyz = rotation.apply(xyz.astype(np.float64)) + translation
        flattened_base = _flatten_floor_points(
            base_xyz, self.octomap_flatten_band_m, self.octomap_flatten_percentile
        )
        sensor_xyz = rotation.inv().apply(flattened_base - translation)
        return sensor_xyz.astype(np.float32)

    def _filter_octomap_cloud(self, msg):
        """Denoise every relayed frame, and additionally drop the targeted
        component's YOLO mask from it while a pick exclusion is active.

        Denoising always runs, not just during pick exclusion — a flying-
        pixel edge point bakes into the octomap exactly like a real object
        does (single hit -> occupied, no partial erase), so a stray point in
        mid-air during any observation is just as permanent as the object
        it's mistaken for.
        """
        offsets = {field.name: field.offset for field in msg.fields}
        if not {"x", "y", "z"} <= offsets.keys():
            return msg  # 예상 못한 필드 구성이면 거르지 않고 그냥 흘려보낸다

        n_points = len(msg.data) // msg.point_step
        if n_points == 0:
            return msg

        # ponytail: x,y,z 가 연속 float32 라고 가정한다 (RealSense 정렬
        # 클라우드의 표준 레이아웃). 다른 레이아웃이면 이 assumption 이
        # _mask_cloud_polygon/_remove_outlier_points 안에서 그냥 조용히
        # 틀린 값을 낼 수 있다 — sensors_3d.yaml 이 가리키는 토픽이 실제로
        # 표준 XYZ 레이아웃인지 의심되면 offsets 를 로그로 찍어서 확인할 것.
        xyz = np.ndarray(
            (n_points, 3),
            dtype=np.float32,
            buffer=msg.data,
            strides=(msg.point_step, 4),
            offset=offsets["x"],
        )

        # 그레이징 앵글 계단 노이즈: 바닥/트레이 면 높이를 base_frame 기준
        # 으로 눌러 편다. 원본 xyz(센서 프레임)는 건드리지 않고, 뒤에서
        # kept 행만 이 보정값으로 덮어쓴다.
        flat_xyz = xyz
        if self.octomap_flatten_floor_enabled:
            flat_xyz = self._flatten_floor_in_sensor_frame(xyz, msg.header)

        keep = _remove_outlier_points(
            flat_xyz, self.octomap_denoise_voxel_m, self.octomap_denoise_min_neighbors
        )

        if self._octomap_exclusion_component is not None:
            # masking_maps는 신선도 체크 "뒤"에 읽어야 한다. motion.py는
            # MultiThreadedExecutor(2 threads)로 돌아서 이 사이에
            # _detection_callback이 다른 스레드에서 _latest_detection_masks를
            # 통째로 교체할 수 있다 — 먼저 읽어두면 신선도 체크는 방금 갱신된
            # (마스크 있음) 상태를 보고 통과시키는데 정작 쓰는 값은 그 전에
            # 읽은 옛 스냅숏(None)이라 _mask_cloud_polygon이 None을 순회하며
            # 죽는 경합이 생긴다.
            fresh = (
                self._has_fresh_exclusion_mask() and self._camera_intrinsics is not None
            )
            masking_maps = (
                self._latest_detection_masks.get(self._octomap_exclusion_component)
                if fresh
                else None
            )
            if not masking_maps:
                self.logger.warn(
                    "옥토맵 예외 미적용(디노이즈만 적용): "
                    f"component={self._octomap_exclusion_component}, "
                    f"mask={'없음' if not masking_maps else '있음'}, "
                    f"intrinsics={'없음' if self._camera_intrinsics is None else '있음'}",
                    throttle_duration_sec=2.0,
                )
            else:
                keep &= _mask_cloud_polygon(
                    flat_xyz,
                    masking_maps,
                    self._camera_intrinsics,
                    self._camera_width,
                    self._camera_height,
                    self.octomap_exclusion_mask_padding_px,
                )
                self.logger.info(
                    "옥토맵 예외 적용: "
                    f"component={self._octomap_exclusion_component}, "
                    f"instances={len(masking_maps)}",
                    throttle_duration_sec=2.0,
                )

        kept = int(np.count_nonzero(keep))
        self.logger.info(
            f"옥토맵 디노이즈: total={n_points}, kept={kept}, dropped={n_points - kept}",
            throttle_duration_sec=5.0,
        )
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            n_points, msg.point_step
        )
        filtered_raw = raw[keep]

        if flat_xyz is not xyz:
            # 평면 펴기로 좌표가 바뀐 경우, 살아남은 행의 x,y,z 바이트만
            # 보정값으로 덮어쓴다. filtered_raw 는 불리언 인덱싱이 만든
            # 새 배열(쓰기 가능)이라 msg.data 원본은 그대로 안전하다.
            xyz_kept_view = np.ndarray(
                (kept, 3),
                dtype=np.float32,
                buffer=filtered_raw,
                strides=(msg.point_step, 4),
                offset=offsets["x"],
            )
            xyz_kept_view[...] = flat_xyz[keep]

        filtered = PointCloud2()
        filtered.header = msg.header
        filtered.height = 1
        filtered.width = kept
        filtered.fields = msg.fields
        filtered.is_bigendian = msg.is_bigendian
        filtered.point_step = msg.point_step
        filtered.row_step = msg.point_step * filtered.width
        filtered.is_dense = msg.is_dense
        filtered.data = filtered_raw.tobytes()
        return filtered

    def set_octomap_mapping(self, enabled):
        """Open/close the point-cloud gate that feeds MoveIt's octomap.

        Only open it while the arm is parked. The camera is eye-in-hand, so a
        cloud captured mid-motion is registered with the wrong TF and smears
        voxels across the workspace.
        """
        if not self.octomap_enabled:
            return False

        enabled = bool(enabled)
        if enabled and self._octomap_frozen:
            # 지도 확정 후에는 어떤 호출자가 열려고 해도 열지 않는다. 닫는
            # 방향(False)은 그대로 통과 — 이미 닫혀 있어 사실상 무해하다.
            self.logger.info(
                "Octomap 확정 상태라 게이트를 다시 열지 않는다",
                throttle_duration_sec=10.0,
            )
            return False

        if enabled != self._octomap_mapping:
            self.logger.info(
                f"Octomap mapping {'ON' if enabled else 'OFF'} "
                f"({self.octomap_cloud_in} -> {self.octomap_cloud_out})"
            )
        self._octomap_mapping = enabled
        return enabled

    def clear_octomap(self):
        """Drop every voxel MoveIt has collected.

        Voxels are never un-occupied by a new cloud that simply no longer sees
        them, so a picked-up object would haunt the map forever. Clear right
        before re-opening the gate at an observation/inspection pose; never
        during a pick, or the map the arm is avoiding disappears.
        """
        if not self.octomap_enabled:
            return False

        if self._octomap_frozen:
            # 확정된 지도를 지우면 그 뒤 구간이 장애물 없는 빈 지도로 계획한다.
            # 다시 채울 촬영 기회가 없으므로(작업당 2회로 끝) 거부한다.
            self.logger.info("Octomap 확정 상태라 clear 요청을 무시한다")
            return False

        future = self._clear_octomap_client.call_async(Empty.Request())
        try:
            self._wait_future(future, self.server_timeout, "ClearOctomap")
        except Exception as error:
            self.logger.warning(f"ClearOctomap failed: {error}")
            return False

        self.logger.info("Octomap cleared")
        return True

    def _allow_octomap_collisions(self):
        """Excuse the gripper links from octomap collisions, once, at startup.

        MoveIt keeps one special ACM entry named "<octomap>" covering every
        voxel. Allowing it against the gripper links lets the fingers reach
        into the target object's voxels while the upper arm still avoids
        everything the camera mapped.

        The ACM has to be read first: a PlanningScene diff carrying a non-empty
        ACM *replaces* the whole matrix, which would wipe the SRDF
        self-collision pairs.
        """
        if not self.octomap_enabled or not self.octomap_allowed_links:
            return False

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get_planning_scene_client.call_async(request)
        response = self._wait_future(
            future, self.server_timeout, "GetPlanningScene(ACM)"
        )

        acm = response.scene.allowed_collision_matrix
        if not acm.entry_names:
            raise RuntimeError(
                "Planning scene returned an empty ACM; "
                "refusing to overwrite self-collision pairs"
            )

        unknown = [
            name
            for name in self.octomap_allowed_links
            if name not in acm.entry_names
        ]
        if unknown:
            # A typo here fails silently otherwise: the entry is created, matches
            # no link, and every grasp keeps colliding with the object voxels.
            self.logger.warning(
                f"octomap.allowed_collision_links not in the robot ACM: {unknown}"
            )

        names, matrix = merge_octomap_acm(
            list(acm.entry_names),
            [list(entry.enabled) for entry in acm.entry_values],
            self.octomap_allowed_links,
        )

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix.entry_names = names
        scene.allowed_collision_matrix.entry_values = [
            AllowedCollisionEntry(enabled=row) for row in matrix
        ]
        scene.allowed_collision_matrix.default_entry_names = list(
            acm.default_entry_names
        )
        scene.allowed_collision_matrix.default_entry_values = list(
            acm.default_entry_values
        )

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_future = self._apply_planning_scene_client.call_async(apply_request)
        apply_response = self._wait_future(
            apply_future, self.server_timeout, "ApplyPlanningScene(octomap ACM)"
        )

        if apply_response is None or not apply_response.success:
            raise RuntimeError("Failed to allow octomap collisions for the gripper")

        self.logger.info(
            "Octomap collisions allowed for "
            f"{len(self.octomap_allowed_links)} links: "
            f"{self.octomap_allowed_links}"
        )
        return True

    # ------------------------------------------------------------------
    # MoveIt joint planning (replacement for movej)
    # ------------------------------------------------------------------

    def move_joint(self, joint_deg, velocity_scale=None, acceleration_scale=None):
        # eye-in-hand: a cloud captured while moving lands on the wrong TF and
        # smears voxels. Stop accumulating; the existing map stays and is avoided.
        self.set_octomap_mapping(False)

        values = list(joint_deg)
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"joint target must have {len(self.joint_names)} values, got {len(values)}"
            )
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("joint target contains non-finite values")

        vel_scale = self._clamp_scale(
            self.default_joint_velocity_scale
            if velocity_scale is None
            else velocity_scale
        )
        acc_scale = self._clamp_scale(
            self.default_joint_acceleration_scale
            if acceleration_scale is None
            else acceleration_scale
        )

        constraints = Constraints()
        constraints.name = "kit_robot_joint_goal"
        for joint_name, value_deg in zip(self.joint_names, values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = math.radians(float(value_deg))
            joint_constraint.tolerance_above = self.joint_tolerance_rad
            joint_constraint.tolerance_below = self.joint_tolerance_rad
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True  # current MoveIt planning-scene state
        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id

        # plan_only=False makes move_group both plan and execute through its
        # configured ros2_control trajectory controller.
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1
        # Preserve the currently monitored planning scene instead of replacing it.
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.logger.info(
            f"MoveIt joint goal: {values}, vel_scale={vel_scale:.3f}, "
            f"acc_scale={acc_scale:.3f}"
        )

        send_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future, self.server_timeout, "sending MoveGroup goal"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected")

        result_future = goal_handle.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future, self.motion_timeout, "MoveGroup execution"
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise

        result = wrapped_result.result
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "MoveGroup plan/execute failed: " + self._error_name(code)
            )
        return 0

    def _move_named_position(self, name):
        """Move to a named position from motion.yaml.

        Supported types:
          - joint: [j1, j2, j3, j4, j5, j6] in degrees
          - cartesian/pose: [x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]
            where A/B/C use the Doosan-compatible intrinsic ZYZ Euler convention.
        """
        if name not in self.positions:
            raise KeyError(f"Unknown named position: {name}")

        config = self.positions[name]
        pose_type = str(config.get("type", "")).strip().lower()
        target = list(config.get("pos", []))
        vel_scale, acc_scale = self._joint_scales(config)

        if pose_type == "joint":
            if len(target) != len(self.joint_names):
                raise ValueError(
                    f"{name}.pos must contain {len(self.joint_names)} joint values"
                )

            self.logger.info(
                f"Move named position '{name}' as joint target: {target}"
            )
            return self.move_joint(
                target,
                velocity_scale=vel_scale,
                acceleration_scale=acc_scale,
            )

        if pose_type in ("cartesian", "pose"):
            if len(target) != 6:
                raise ValueError(
                    f"{name}.pos must be "
                    "[x_mm, y_mm, z_mm, A_deg, B_deg, C_deg]"
                )

            position_tolerance_mm = float(
                config.get("position_tolerance_mm", 2.0)
            )
            orientation_tolerance_deg = float(
                config.get("orientation_tolerance_deg", 2.0)
            )

            self.logger.info(
                f"Move named position '{name}' as Cartesian target: {target}, "
                f"position_tolerance={position_tolerance_mm:.2f} mm, "
                f"orientation_tolerance={orientation_tolerance_deg:.2f} deg"
            )

            return self.move_pose(
                target,
                velocity_scale=vel_scale,
                acceleration_scale=acc_scale,
                position_tolerance_mm=position_tolerance_mm,
                orientation_tolerance_deg=orientation_tolerance_deg,
            )

        raise ValueError(
            f"Unsupported position type for '{name}': {pose_type!r}. "
            "Expected 'joint' or 'cartesian'."
        )

    def move_home(self):
        return self._move_named_position("home")

    def move_to_observation_pose(self):
        # observation_pose는 motion.yaml의 Cartesian [x,y,z,A,B,C] 목표다.
        # A/B/C는 intrinsic ZYZ Euler로 저장하고, _move_named_position()
        # -> move_pose() -> _pose6_to_ros_pose()에서 quaternion으로 변환해
        # MoveIt pose goal로 전달한다.
        #
        # 게이트 요청은 additive — 절대 먼저 지우지 않는다. 작업당 한 번뿐인
        # move_to_inspection_pose() 가 쌓아둔 키팅 트레이 voxel 이 살아남아야
        # 컴포넌트 2번 이후에도 "트레이 피하기"가 동작한다.
        #
        # 키팅 트레이 스캔 직후 Controller 가 freeze_octomap() 을 걸어두므로
        # 여기서 부르는 set_octomap_mapping(True) 는 실제로는 항상 무시된다 —
        # 관찰 자세에서는 새 voxel 을 쌓지 않는다.
        #
        # ponytail: a picked-up object leaves a ghost voxel behind (occupancy
        # only grows without an explicit clear), which can make a since-cleared
        # spot on the pick tray look blocked. Fails safe (overly cautious, not
        # collision-prone). If that starts blocking real plans, clear just the
        # vacated grasp footprint here instead of reintroducing a full clear.
        result = self._move_named_position("observation_pose")
        self._wait_for_exclusion_mask()
        self.set_octomap_mapping(True)
        return result

    def freeze_octomap(self):
        """Seal the map: no more accumulation, no more clearing, for this task.

        Called once per task by Controller right after the single seeding
        scan (the keating tray at inspection_pose). Every later move —
        observation, pick, place, final inspection — plans against exactly
        this map. Freezing rather than just closing the gate matters because
        several call sites (`move_to_observation_pose`,
        `move_to_inspection_pose`) reopen the gate on their own; the flag is
        what stops all of them at once.
        """
        if not self.octomap_enabled:
            return False

        self.set_octomap_mapping(False)
        self._octomap_frozen = True
        self.logger.info("Octomap 확정: 이후 촬영·삭제 없이 이 지도만 사용한다")
        return True

    def _wait_for_exclusion_mask(self, timeout_sec=2.0):
        """예외 걸린 컴포넌트가 있으면, 그 클래스의 첫 YOLO 탐지가 도착할 때까지
        게이트를 열지 않고 기다린다.

        게이트를 먼저 열고 마스크를 기다리면, 그 사이 들어오는 프레임은
        무필터로 중계되어 그 물체의 voxel 이 그대로 박힌다 — 부분 삭제
        API가 없어서 한 번 박히면 이번 관찰에서는 영영 못 뺀다. timeout
        안에 안 와도 mapping 상태는 열어 두되, cloud callback이 새 마스크가
        없는 프레임을 버린다. 감지 실패가 대상 물체 voxel을 만들지는 않는다.
        """
        component = self._octomap_exclusion_component
        if component is None:
            return
        deadline = time.monotonic() + timeout_sec
        while not Motion._has_fresh_exclusion_mask(self):
            if time.monotonic() >= deadline:
                self.logger.warn(
                    f"옥토맵 예외 대기 타임아웃({timeout_sec}s): "
                    f"component={component} 새 탐지가 없음; 해당 cloud는 계속 버림"
                )
                return
            time.sleep(0.05)

    def move_to_inspection_pose(self, clear_before=False):
        # Clearing refresh: used once per task by Controller (before the first
        # observation, to seed the place-area map) and once more at the very end
        # (final inspection). The end-of-task call keeps moving against the
        # *current* map (clear_before=False) — those voxels are this task's own
        # placed components, still real obstacles for that move.
        #
        # The once-per-task first call is different: it's the very first move
        # of a brand new task, and the map it would otherwise plan against is
        # whatever the *previous* task's octomap left behind (a failed pick,
        # a picked-up ghost voxel, ...) — garbage this task never built and
        # has no way to know about. Clearing before that move, not just after,
        # is what actually fixes "새 작업인데 이전 작업 octomap 때문에 여기로
        # 못 움직임": otherwise the plan to inspection_pose can be blocked by
        # stale voxels that clear_octomap() only wipes *after* the move already
        # failed. Safe to clear first here — the gate is closed for the whole
        # move (see move_joint), so nothing new bakes in between.
        if clear_before:
            # clear_before 는 "새 작업의 첫 이동" 신호다 — 지난 작업이 확정해
            # 잠가둔 지도를 여기서 풀어야 이번 작업 지도를 새로 쌓을 수 있다.
            # 푸는 게 먼저다: 잠긴 상태에서는 clear_octomap() 이 거부된다.
            self._octomap_frozen = False
            self.clear_octomap()
        result = self._move_named_position("inspection_pose")
        self.clear_octomap()
        self.set_octomap_mapping(True)
        return result

    # ------------------------------------------------------------------
    # Current TCP pose: keep old DSR posx-compatible mm + ZYZ-degree format
    # ------------------------------------------------------------------

    def get_current_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.eef_link,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as error:
            raise RuntimeError(
                f"TF lookup failed: {self.base_frame} <- {self.eef_link}: {error}"
            ) from error

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]

        # ZYZ has a mathematical singularity at beta=0/pi. scipy can still return
        # an equivalent Euler triplet; reconstruction of the rotation remains valid.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            rx, ry, rz = Rotation.from_quat(quat).as_euler("ZYZ", degrees=True)

        return [
            float(translation.x * 1000.0),
            float(translation.y * 1000.0),
            float(translation.z * 1000.0),
            float(rx),
            float(ry),
            float(rz),
        ]

    # ------------------------------------------------------------------
    # MoveIt pose-goal planning
    # ------------------------------------------------------------------

    def move_pose(
        self,
        target_pose,
        velocity_scale=None,
        acceleration_scale=None,
        position_tolerance_mm=2.0,
        orientation_tolerance_deg=2.0,
    ):
        """Plan/execute a free-space MoveIt pose goal for ``eef_link``.

        Unlike move_linear(), this does not force a Cartesian straight line.
        OMPL/MoveGroup is free to choose a collision-aware joint-space path
        that reaches the requested Cartesian pose.

        External pose convention:
            [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        where orientation is ZYZ Euler in ``base_frame``.
        """

        self.set_octomap_mapping(False)

        if position_tolerance_mm <= 0.0:
            raise ValueError("position_tolerance_mm must be positive")
        if orientation_tolerance_deg <= 0.0:
            raise ValueError("orientation_tolerance_deg must be positive")

        ros_pose = self._pose6_to_ros_pose(target_pose)

        vel_scale = self._clamp_scale(
            self.default_joint_velocity_scale
            if velocity_scale is None
            else velocity_scale
        )
        acc_scale = self._clamp_scale(
            self.default_joint_acceleration_scale
            if acceleration_scale is None
            else acceleration_scale
        )

        constraints = Constraints()
        constraints.name = "kit_robot_pose_goal"

        # Position goal: a small box around the requested XYZ.
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.eef_link
        position_constraint.weight = 1.0

        tolerance_m = float(position_tolerance_mm) / 1000.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            2.0 * tolerance_m,
            2.0 * tolerance_m,
            2.0 * tolerance_m,
        ]

        box_pose = Pose()
        box_pose.position.x = ros_pose.position.x
        box_pose.position.y = ros_pose.position.y
        box_pose.position.z = ros_pose.position.z
        box_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(box_pose)
        constraints.position_constraints.append(position_constraint)

        # Orientation goal: keep the requested quaternion within a small tolerance.
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.eef_link
        orientation_constraint.orientation = ros_pose.orientation

        orientation_tolerance_rad = math.radians(
            float(orientation_tolerance_deg)
        )
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.request.goal_constraints = [constraints]
        goal.request.start_state.is_diff = True

        if self.pipeline_id:
            goal.request.pipeline_id = self.pipeline_id
        if self.planner_id:
            goal.request.planner_id = self.planner_id

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.1
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.logger.info(
            f"MoveIt pose goal: {list(target_pose)}, "
            f"vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f}"
        )

        send_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future,
            self.server_timeout,
            "sending MoveGroup pose goal",
        )

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup pose goal was rejected")

        result_future = goal_handle.get_result_async()

        try:
            wrapped_result = self._wait_future(
                result_future,
                self.motion_timeout,
                "MoveGroup pose execution",
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise

        result = wrapped_result.result
        code = int(result.error_code.val)

        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "MoveGroup pose plan/execute failed: "
                + self._error_name(code)
            )

        return 0

    # ------------------------------------------------------------------
    # Cartesian straight-line planning (replacement for movel)
    # ------------------------------------------------------------------

    def move_linear(self, target_pose, vel=100, acc=200, avoid_collisions=True):
        self.set_octomap_mapping(False)

        if vel <= 0 or acc <= 0:
            raise ValueError("vel and acc must be positive")

        ros_pose = self._pose6_to_ros_pose(target_pose)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self._moveit_node.get_clock().now().to_msg()
        request.start_state.is_diff = True  # current planning-scene state
        request.group_name = self.group_name
        request.link_name = self.eef_link
        request.waypoints = [ros_pose]
        request.max_step = self.cartesian_max_step_m
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = bool(avoid_collisions)

        # GetCartesianPath in MoveIt Jazzy can directly limit Cartesian speed.
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = self._clamp_scale(
            float(acc) / self.cartesian_acc_reference_mm_s2
        )
        request.cartesian_speed_limited_link = self.eef_link
        request.max_cartesian_speed = float(vel) / 1000.0  # mm/s -> m/s

        self.logger.info(
            f"MoveIt Cartesian goal: {list(target_pose)}, "
            f"speed={float(vel):.1f} mm/s"
        )

        future = self._cartesian_client.call_async(request)
        response = self._wait_future(
            future, self.motion_timeout, "GetCartesianPath"
        )
        if response is None:
            raise RuntimeError("GetCartesianPath returned no response")

        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Cartesian planning failed: " + self._error_name(code)
            )

        if response.fraction < self.cartesian_min_fraction:
            raise RuntimeError(
                "Cartesian path incomplete: "
                f"fraction={response.fraction:.3f} < {self.cartesian_min_fraction:.3f}"
            )

        # If the requested pose is effectively the current pose MoveIt may return
        # a valid empty trajectory. Treat that as success rather than sending it.
        points = response.solution.joint_trajectory.points
        if not points:
            self.logger.info("Cartesian target already satisfied (empty trajectory).")
            return 0

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal.controller_names = []  # let MoveIt choose configured controller(s)

        send_future = self._execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(
            send_future, self.server_timeout, "sending ExecuteTrajectory goal"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ExecuteTrajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        try:
            wrapped_result = self._wait_future(
                result_future, self.motion_timeout, "Cartesian trajectory execution"
            )
        except Exception:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            raise

        code = int(wrapped_result.result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "ExecuteTrajectory failed: " + self._error_name(code)
            )
        return 0

    def _move_pose_stepped(self, target_pose, step_mm=50.0):
        """OMPL(move_pose)로 target_pose까지 여러 홉에 걸쳐 이동한다.

        move_pose 한 번으로 트레이 상공-바닥처럼 먼 거리를 뛰면 narrow
        passage(좁은 통로) 샘플링 실패로 PLANNING_FAILED가 잦다. move_linear
        (GetCartesianPath)는 fraction >= cartesian_min_fraction을 못 채우면
        그냥 실패해서 이 경로엔 안 맞는다. 대신 짧은 구간으로 쪼개 각 홉을
        OMPL이 풀기 쉬운 문제로 만든다 — 거의 직선이라 한 홉씩은 잘 풀린다.
        """
        current_pose = self.get_current_pose()
        distance_mm = math.dist(current_pose[:3], target_pose[:3])
        steps = max(1, math.ceil(distance_mm / step_mm))

        for i in range(1, steps + 1):
            waypoint = [
                current_pose[j] + (target_pose[j] - current_pose[j]) * i / steps
                for j in range(6)
            ]
            self.move_pose(waypoint)

    # ------------------------------------------------------------------
    # High-level pick/place API kept identical to old Motion
    # ------------------------------------------------------------------

    def pick_component(self, component_name, target_pose, vel=80, acc=160):
        pose = list(target_pose)
        if len(pose) != 6:
            raise ValueError("target_pose must be [x, y, z, rx, ry, rz]")

        params = self.grasp_params.get(
            component_name, self.grasp_params["_default"]
        )
        open_width = params["width"]
        grip_force = params["force"]
        approach_height = params["approach"]

        pick_pose_down = pose.copy()
        pick_pose_up = pose.copy()
        pick_pose_up[2] += approach_height

        self.logger.info(
            f"Pick component: {component_name}, width={open_width}, "
            f"force={grip_force}, approach={approach_height}"
        )

        # ACM은 그리퍼 링크만 옥토맵과 충돌 면제해서, 손목/팔뚝이 접근하는
        # 경로까지는 못 풀어준다 — 그래서 이 물체의 YOLO 마스크에 해당하는
        # depth 포인트는 옥토맵에 애초에 들어가지 않도록 걸러낸다(부분 삭제
        # API가 없어서, 한 번 voxel로 박히면 clear_octomap()으로 맵 전체를
        # 비우는 수밖에 없고, 그건 키팅 트레이 지도까지 같이 날린다).
        # Controller가 move_to_observation_pose() 전에 이미 걸어뒀어야 첫
        # 프레임부터 효과가 있다 — 여기서 또 거는 건 그게 빠졌을 때의 안전망.
        self.set_octomap_exclusion_component(component_name)

        try:
            for attempt_index in range(5):
                self.rg.move_gripper(open_width, force_val=grip_force)
                time.sleep(2.0)

                self.move_pose(pick_pose_up)
                time.sleep(0.5)
                self.move_linear(pick_pose_down, vel=vel, acc=acc)

                self.rg.close_gripper(force_val=grip_force)
                time.sleep(5.0)

                self.move_linear(pick_pose_up, vel=vel, acc=acc, avoid_collisions=False)
                time.sleep(1.0)

                gripper_status = self.rg.get_status()
                grip_detected = bool(gripper_status[1])

                if grip_detected:
                    self.logger.info("Successfully gripped object")
                    return True
                else:
                    print(f"{attempt_index + 1} try, Failed to grip object")

                self.logger.warning(
                    f"Grasp check failed ({attempt_index + 1}/5)"
                )

            self.logger.error("Failed to grip object in all 5 attempts")
            return False
        finally:
            # 성공하든 실패하든 이 물체 자리를 영영 옥토맵에서 빼놓지 않는다 —
            # 실패하면 남아있는 물체를 다시 장애물로 취급해야 안전하다.
            self.clear_octomap_exclusion()

    def place_component(self, component_name, slot_name, approach_height=350):
        if slot_name not in self.place_slots:
            raise ValueError(f"Unknown place slot: {slot_name}")

        place_pose_down = list(self.place_slots[slot_name]["pos"])
        if len(place_pose_down) != 6:
            raise ValueError(
                f"{slot_name} pose must be [x, y, z, rx, ry, rz]"
            )

        # PLACE 목표 위치보다 Z 방향으로 approach_height만큼 높은 안전 위치.
        place_pose_up = place_pose_down.copy()
        place_pose_up[2] += approach_height

        # ---------------------------------------------------------
        # PLACE 좌표 확인
        # ---------------------------------------------------------
        current_pose = self.get_current_pose()

        self.logger.info(
            f"[PLACE] component={component_name}, slot={slot_name}"
        )

        self.logger.info(
            "[PLACE] current_pose = "
            f"[x={current_pose[0]:.2f}, "
            f"y={current_pose[1]:.2f}, "
            f"z={current_pose[2]:.2f}, "
            f"rx={current_pose[3]:.2f}, "
            f"ry={current_pose[4]:.2f}, "
            f"rz={current_pose[5]:.2f}]"
        )

        self.logger.info(
            "[PLACE] place_pose_up = "
            f"[x={place_pose_up[0]:.2f}, "
            f"y={place_pose_up[1]:.2f}, "
            f"z={place_pose_up[2]:.2f}, "
            f"rx={place_pose_up[3]:.2f}, "
            f"ry={place_pose_up[4]:.2f}, "
            f"rz={place_pose_up[5]:.2f}]"
        )

        self.logger.info(
            "[PLACE] place_pose_down = "
            f"[x={place_pose_down[0]:.2f}, "
            f"y={place_pose_down[1]:.2f}, "
            f"z={place_pose_down[2]:.2f}, "
            f"rx={place_pose_down[3]:.2f}, "
            f"ry={place_pose_down[4]:.2f}, "
            f"rz={place_pose_down[5]:.2f}]"
        )

        self.logger.info(
            f"[PLACE] approach_height={approach_height} mm"
        )

        # ---------------------------------------------------------
        # 1. PLACE 상공 안전 위치로 이동
        #    Cartesian 직선 경로를 강제하지 않고 MoveIt이
        #    collision-aware joint-space 경로를 계획한다.
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_pose -> place_pose_up: {place_pose_up}"
        )

        self.move_pose(place_pose_up)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after move_pose(up) = {current_pose}"
        )

        time.sleep(0.5)

        # ---------------------------------------------------------
        # 2. PLACE 위치까지 하강
        #    move_linear(GetCartesianPath)는 fraction 기준을 못 채우면
        #    "움직일 수 없는 경로"로 바로 실패해서 여기선 안 쓴다.
        #    move_pose 한 번의 큰 점프도 narrow passage 샘플링 실패
        #    (PLANNING_FAILED)가 잦아서, _move_pose_stepped로 짧게
        #    쪼개 내려간다.
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] move_pose(stepped) -> place_pose_down: {place_pose_down}"
        )

        self._move_pose_stepped(place_pose_down)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after move_pose(down) = {current_pose}"
        )

        # ---------------------------------------------------------
        # 3. 물체 놓기
        # ---------------------------------------------------------
        self.logger.info("[PLACE] Opening gripper")
        self.rg.open_gripper()
        time.sleep(2.0)

        # ---------------------------------------------------------
        # 4. 다시 상공으로 상승 (하강과 동일하게 stepped move_pose)
        # ---------------------------------------------------------
        self.logger.info(
            f"[PLACE] retreat move_pose(stepped) -> place_pose_up: {place_pose_up}"
        )

        self._move_pose_stepped(place_pose_up)

        current_pose = self.get_current_pose()
        self.logger.info(
            f"[PLACE] actual pose after retreat = {current_pose}"
        )

        self.logger.info(
            f"Complete place: {component_name} -> {slot_name}"
        )

        return True

    def recover_to_safe_pose(self):
        self.logger.warning("Recovering from task failure")
        self.rg.open_gripper()
        time.sleep(2.0)
        self.logger.info("Recovery complete; current robot pose is preserved")

    def shutdown(self):
        """Stop the private MoveIt executor/node before rclpy.shutdown()."""
        try:
            if hasattr(self, "_executor"):
                self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            try:
                self._executor.cancel()
            except Exception:
                pass

        try:
            if hasattr(self, "_spin_thread") and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if hasattr(self, "_moveit_node"):
                self._moveit_node.destroy_node()
        except Exception:
            pass
