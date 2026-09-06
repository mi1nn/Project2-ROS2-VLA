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

# `DSR_ROBOT2` 를 import 하지 않는 순수 계산 노드다. 로봇 자세는 controller 가
# request 에 담아 보낸다 (02-interfaces.md §2.5) — 로봇/DR_init 없이도 이 파일은 그대로 돈다.

PACKAGE_NAME = "kit_robot"

# 검출 토픽은 상시 발행 + 최신 검출만 의미 있음 → BEST_EFFORT, depth=1 (02-interfaces.md §2.3).
# 구독 쪽도 동일 QoS 라야 매칭된다. 신선도는 max_age_sec 로 이 노드가 검사하므로
# 트랜스포트의 재전송 보장은 불필요하다.
DETECTION_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

# 레퍼런스 기본값. 품목별 실측치는 resource/grasp_params.json 이 있으면 그쪽 우선 (Day 8 튜닝).
DEPTH_OFFSET = -35.0
MIN_DEPTH = 2.0     # 테이블 관통 방지 하한
DEFAULT_MAX_AGE_SEC = 1.0

# mm, 실측 후 조정 (03-system-flow.md §4.3)
WORKSPACE = {"x": (200.0, 800.0), "y": (-400.0, 400.0), "z": (0.0, 500.0)}


# ---------------------------------------------------------------------------
# 순수 함수 — 로봇/ROS 없이 self-check 가능 (03-system-flow.md §4.4)
# ---------------------------------------------------------------------------

def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    """posx(mm, deg, ZYZ 오일러) → 4x4 동차변환행렬. 레퍼런스 robot_control.py 그대로."""
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def transform_to_base(camera_xyz, gripper2cam, robot_posx):
    """카메라 좌표 → 베이스 좌표. eye-in-hand 이므로 촬영 시점의 robot_posx 를 써야 한다."""
    x, y, z, rx, ry, rz = robot_posx
    base2gripper = get_robot_pose_matrix(x, y, z, rx, ry, rz)
    base2cam = base2gripper @ gripper2cam
    coord = np.append(np.asarray(camera_xyz, dtype=float), 1.0)
    return (base2cam @ coord)[:3]


def mask_min_width_axis_angle(polygon_flat):
    """masking_map(평탄화된 [x1,y1,x2,y2,...] 픽셀 폴리곤)에서 최소 외접 사각형의
    짧은 변 각도(deg)를 구한다. 평행 조 그리퍼는 가장 좁은 단면을 가로질러 잡아야
    안정적이다 (03-system-flow.md §4.2). 점이 3개 미만이거나 폴리곤이 퇴화되면 None —
    호출부는 None 을 관찰 자세 rz 폴백 신호로 쓴다.
    """
    if polygon_flat is None or len(polygon_flat) < 6:
        return None
    pts = np.asarray(polygon_flat, dtype=np.float32).reshape(-1, 2)
    (_, _), (w, h), angle = cv2.minAreaRect(pts)
    if w == 0 or h == 0:
        return None
    return float(angle) if w < h else float(angle) + 90.0


def compute_rz(observe_rz, pixel_angle_deg):
    """마스크 최소폭 축 각도를 관찰 자세 rz 위에 얹어 최종 rz 를 근사한다.

    전제: rx,ry 는 관찰 자세 그대로(수직 하향 고정)이므로 카메라 광축과 그리퍼
    접근축이 같은 평면을 공유한다 — 픽셀 평면 회전을 그대로 rz 축 회전으로 보는
    근사다. 부호/반전은 실제 캘리브레이션 후 확인이 필요하다 (03-system-flow.md §4.2).
    """
    return observe_rz + pixel_angle_deg


def compute_target_pose(camera_xyz, masking_map, robot_posx, gripper2cam, z_offset, workspace):
    """검출 하나(카메라 좌표 + 폴리곤)를 베이스 좌표 파지 자세로 변환한다.

    반환: (pose[6] | None, error_code | None). pose 가 None 이면 error_code 에 사유가 담긴다.
    """
    base_xyz = transform_to_base(camera_xyz, gripper2cam, robot_posx)
    x, y, z = base_xyz
    z = max(z + z_offset, MIN_DEPTH)

    wx, wy, wz = workspace["x"], workspace["y"], workspace["z"]
    if not (wx[0] <= x <= wx[1] and wy[0] <= y <= wy[1] and wz[0] <= z <= wz[1]):
        return None, "out_of_workspace"

    rx, ry, observe_rz = robot_posx[3], robot_posx[4], robot_posx[5]
    pixel_angle = mask_min_width_axis_angle(masking_map)
    rz = compute_rz(observe_rz, pixel_angle) if pixel_angle is not None else observe_rz

    return [float(x), float(y), float(z), float(rx), float(ry), float(rz)], None


def detection_key(obj):
    """검출 하나를 식별하는 키. `DetectedObject` 에 고유 id 가 없어 픽셀 무게중심으로
    대체한다 (exclude_taken 용). 같은 프레임 안에서 같은 클래스가 정확히 같은 픽셀에
    두 번 잡히진 않는다는 전제 — 더 견고한 식별자가 필요해지면 계약(02-interfaces.md
    §2.4)에 id 필드를 추가하는 걸 검토한다."""
    return f"{obj.centroid_px[0]},{obj.centroid_px[1]}"


def select_candidates(objects, component, exclude_taken):
    """클래스가 일치하고 exclude_taken 에 없는 검출만 남긴다."""
    exclude = set(exclude_taken or [])
    return [o for o in objects if o.class_name == component and detection_key(o) not in exclude]


def inspect_counts(objects, expected_classes, expected_counts):
    """레시피 기대치와 실제 검출 개수를 비교한다 (02-interfaces.md §2.7)."""
    counts = Counter(o.class_name for o in objects)
    actual_counts = [counts.get(c, 0) for c in expected_classes]
    missing = [c for c, exp, act in zip(expected_classes, expected_counts, actual_counts) if act < exp]
    expected_set = set(expected_classes)
    unexpected = sorted(name for name in counts if name not in expected_set)
    ok = actual_counts == list(expected_counts) and not unexpected
    return ok, missing, unexpected, actual_counts


# ---------------------------------------------------------------------------
# 리소스 로더 — 없어도 노드가 죽지 않고 명확한 경고와 함께 대체값을 쓴다
# (Day 3 캘리브레이션 / Day 8 튜닝 전에도 골격 결선을 끝낼 수 있어야 한다, 04-roadmap.md)
# ---------------------------------------------------------------------------

def load_gripper2cam(path, logger=None):
    """hand-eye 행렬 로드. 파일이 없으면 항등행렬로 대체하고 크게 경고한다 —
    이 상태로 나온 좌표는 신뢰할 수 없다."""
    if os.path.isfile(path):
        return np.load(path)
    msg = f"T_gripper2camera.npy 없음 ({path}) — 항등행렬 사용 중, 캘리브레이션 전까지 좌표 신뢰 불가"
    (logger.error if logger else print)(msg)
    return np.eye(4)


def load_grasp_z_offsets(path, logger=None):
    """resource/grasp_params.json 에서 클래스별 z_offset 만 뽑는다. width/force 등 나머지
    필드는 motion.py/grasp.py 몫. 파일이 없으면 빈 dict — 호출부가 DEPTH_OFFSET 기본값을 쓴다."""
    if not os.path.isfile(path):
        if logger:
            logger.warn(f"grasp_params.json 없음 ({path}) — 전 품목 DEPTH_OFFSET 기본값 사용")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return {name: params["z_offset"] for name, params in raw.items() if "z_offset" in params}


# ---------------------------------------------------------------------------
# 노드
# ---------------------------------------------------------------------------

class PositionEstimationNode(Node):
    def __init__(self):
        super().__init__('position_estimation_node')

        resource_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), "resource")
        self.gripper2cam = load_gripper2cam(
            os.path.join(resource_dir, "T_gripper2camera.npy"), self.get_logger())
        self.z_offsets = load_grasp_z_offsets(
            os.path.join(resource_dir, "grasp_params.json"), self.get_logger())
        self.workspace = WORKSPACE

        self.latest = None  # 최신 DetectionArray. 요청과 무관하게 토픽에서 계속 갱신된다.
        self.create_subscription(
            DetectionArray, '/detection/objects', self._on_detections, DETECTION_QOS)
        self.create_service(GetComponentPose, '/get_component_pose', self.get_component_pose_callback)
        self.create_service(InspectKit, '/inspect_kit', self.inspect_kit_callback)

        self.get_logger().info("PositionEstimationNode initialized.")

    def _on_detections(self, msg):
        self.latest = msg

    def _age_sec(self):
        now = self.get_clock().now()
        stamp_time = Time.from_msg(self.latest.header.stamp)
        return (now - stamp_time).nanoseconds / 1e9

    def get_component_pose_callback(self, request, response):
        if self.latest is None:
            response.success = False
            response.error_code = "not_detected"
            return response

        # 최신성 검사가 제일 먼저다 — 팔 이동 전 프레임에 지금 posx 를 곱하면
        # 좌표가 통째로 틀린다 (02-interfaces.md §2.6).
        max_age = request.max_age_sec if request.max_age_sec > 0 else DEFAULT_MAX_AGE_SEC
        if self._age_sec() > max_age:
            response.success = False
            response.error_code = "stale"
            return response

        candidates = select_candidates(self.latest.objects, request.component, request.exclude_taken)
        if not candidates:
            any_class = any(o.class_name == request.component for o in self.latest.objects)
            response.success = False
            response.error_code = "no_candidate" if any_class else "not_detected"
            return response

        z_offset = self.z_offsets.get(request.component, DEPTH_OFFSET)
        robot_posx = list(request.robot_posx)

        in_workspace = []
        for obj in candidates:
            pose, err = compute_target_pose(
                list(obj.camera_xyz), list(obj.masking_map), robot_posx,
                self.gripper2cam, z_offset, self.workspace)
            if err is None:
                in_workspace.append((obj, pose))

        if not in_workspace:
            # 좌표가 작업영역 밖이면 움직이지 않는다 — 어떤 검출을 보고 걸러졌는지는 남긴다.
            response.success = False
            response.error_code = "out_of_workspace"
            response.source = candidates[0]
            return response

        obj, pose = max(in_workspace, key=lambda pair: pair[0].score)
        response.success = True
        response.target_pose = pose
        response.source = obj
        response.error_code = ""
        return response

    def inspect_kit_callback(self, request, response):
        # InspectKit.srv 엔 error_code 가 없지만, 최신성 판정은 GetComponentPose 와
        # 같은 가드를 재사용해야 한다 (02-interfaces.md §2.7). stale 이면 카운팅과
        # 무관하게 ok=False — 우연히 개수가 맞아도 낡은 프레임 기준 통과를 허용하지 않는다.
        response.detection_age = self._age_sec() if self.latest is not None else float("inf")
        max_age = request.max_age_sec if request.max_age_sec > 0 else DEFAULT_MAX_AGE_SEC

        if self.latest is None or response.detection_age > max_age:
            response.ok = False
            response.missing = []
            response.unexpected = []
            response.actual_counts = [0] * len(request.expected_classes)
            return response

        ok, missing, unexpected, actual_counts = inspect_counts(
            self.latest.objects, list(request.expected_classes), list(request.expected_counts))
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


def _demo():
    # 항등 hand-eye 행렬 + 원점 자세면 카메라 좌표가 그대로 베이스 좌표여야 한다 (03-system-flow.md §4.4)
    T = np.eye(4)
    assert np.allclose(transform_to_base([100, 0, 500], T, [0, 0, 0, 0, 0, 0]), [100, 0, 500])
    # 베이스가 x 로 200 평행이동하면 결과도 200 만큼 이동
    assert np.allclose(transform_to_base([100, 0, 500], T, [200, 0, 0, 0, 0, 0]), [300, 0, 500])

    # mask 최소폭 축 각도 — 점이 부족하면 폴백 신호(None)
    assert mask_min_width_axis_angle([1, 2, 3]) is None
    assert mask_min_width_axis_angle(None) is None
    angle = mask_min_width_axis_angle([0, 0, 100, 0, 100, 20, 0, 20])
    assert angle is not None and np.isfinite(angle)

    # 작업영역 밖이면 out_of_workspace, 좌표를 그대로 흘리지 않는다
    workspace = WORKSPACE
    robot_posx = [0, 0, 0, 0, 0, 0]
    pose, err = compute_target_pose([100, 0, 500], None, robot_posx, T, -35.0, workspace)
    assert pose is None and err == "out_of_workspace", (pose, err)

    # 작업영역 안이면 z_offset 이 반영되고, 폴리곤이 없으면 rz 는 관찰 자세값 그대로
    pose, err = compute_target_pose([300, 0, 500], None, robot_posx, T, -35.0, workspace)
    assert err is None and pose == [300.0, 0.0, 465.0, 0.0, 0.0, 0.0], (pose, err)

    # exclude_taken 은 이미 집어간 위치를 후보에서 뺀다
    class _Obj:
        def __init__(self, class_name, centroid_px, score):
            self.class_name, self.centroid_px, self.score = class_name, centroid_px, score

    objs = [_Obj("cup_ramen", [10, 10], 0.9), _Obj("cup_ramen", [50, 50], 0.8), _Obj("mask", [30, 30], 0.95)]
    assert len(select_candidates(objs, "cup_ramen", [])) == 2
    remaining = select_candidates(objs, "cup_ramen", ["10,10"])
    assert len(remaining) == 1 and remaining[0].centroid_px == [50, 50]

    # 검사 카운팅: 일치 / 수량 부족+오투입
    ok, missing, unexpected, actual = inspect_counts(objs, ["cup_ramen", "mask"], [2, 1])
    assert ok and missing == [] and unexpected == [] and actual == [2, 1], (ok, missing, unexpected, actual)

    ok, missing, unexpected, actual = inspect_counts(objs, ["cup_ramen"], [3])
    assert not ok and missing == ["cup_ramen"] and unexpected == ["mask"], (ok, missing, unexpected, actual)

    print("ok")


if __name__ == "__main__":
    _demo()
