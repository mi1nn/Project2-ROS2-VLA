"""Octomap 설정 두 파일이 서로 맞는지 검사한다.

topic 이 한 글자만 틀려도 move_group 은 조용히 빈 octomap 을 들고 돈다(에러 없음,
RViz 에 voxel 만 안 보임). 로봇을 띄우기 전에 잡는다.

실행: python3 -m pytest src/kit_robot/test/test_octomap_config.py
"""
from pathlib import Path

import pytest
import yaml

MOVEIT_SENSORS = (
    "src/API/doosan-robot2/dsr_moveit2/dsr_moveit_config_m0609"
    "/config/sensors_3d.yaml"
)
MOTION_CONFIG = "src/kit_robot/config/motion.yaml"


def _workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / MOTION_CONFIG).exists():
            return parent
    pytest.skip("워크스페이스 소스 트리를 찾지 못했다 (설치본만 있는 환경)")


def _load(relative_path):
    path = _workspace_root() / relative_path
    if not path.exists():
        pytest.skip(f"{relative_path} 없음")
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def test_octomap_configs_agree():
    sensors = _load(MOVEIT_SENSORS)
    moveit = _load(MOTION_CONFIG)["motion"]["moveit"]
    octomap = moveit["octomap"]

    names = sensors.get("sensors") or []
    assert names, "sensors_3d.yaml 의 sensors 리스트가 비었다 -> octomap 비활성"

    updaters = [sensors[name] for name in names]
    plugins = [updater["sensor_plugin"] for updater in updaters]
    assert "occupancy_map_monitor/PointCloudOctomapUpdater" in plugins, plugins

    topics = [
        updater["point_cloud_topic"]
        for updater in updaters
        if updater["sensor_plugin"].endswith("PointCloudOctomapUpdater")
    ]
    assert octomap["cloud_out"] in topics, (
        f"motion.yaml octomap.cloud_out={octomap['cloud_out']} 가 "
        f"sensors_3d.yaml point_cloud_topic {topics} 에 없다"
    )

    # octomap_frame 이 keepout box 와 다른 프레임이면 벽과 지도가 어긋난다.
    assert sensors["octomap_frame"] == moveit["base_frame"]

    # 절대 토픽이어야 한다. move_group 은 /dsr01 네임스페이스라 상대 이름은
    # /dsr01/... 로 치환되어 중계 토픽과 만나지 못한다.
    for topic in topics:
        assert topic.startswith("/"), topic

    assert 0.0 < sensors["octomap_resolution"] <= 0.05
    for updater in updaters:
        assert 0.0 < updater["max_range"] <= 3.0, "D435 는 3 m 넘으면 노이즈뿐"


def test_octomap_acm_links_spare_the_upper_arm():
    """그리퍼만 voxel 통과를 허용해야 회피가 남는다.

    여기에 link_1~link_5 나 base 가 들어가면 octomap 은 그려지기만 하고 아무것도
    막지 못한다 — 기능 전체가 조용히 무력화된다.
    """
    octomap = _load(MOTION_CONFIG)["motion"]["moveit"]["octomap"]
    links = octomap["allowed_collision_links"]

    assert links, "allowed_collision_links 가 비면 파지가 계획되지 않는다"
    assert octomap["get_planning_scene_service"].startswith("/")

    forbidden = [name for name in links if name in {
        "base", "base_link", "link_1", "link_2", "link_3", "link_4", "link_5",
    }]
    assert not forbidden, f"상완·본체가 voxel 통과 허용에 들어있다: {forbidden}"

    # 손가락이 빠지면 물체를 감쌀 수 없다.
    assert any("inner_finger" in name for name in links), links


def test_merge_octomap_acm_keeps_existing_pairs():
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    # link_a <-> link_b 는 SRDF 자기충돌 허용 쌍이라고 가정한다.
    names = ["link_a", "link_b", "link_6"]
    matrix = [
        [False, True, False],
        [True, False, False],
        [False, False, False],
    ]

    merged_names, merged = motion.merge_octomap_acm(
        names, matrix, ["link_6", "rg2_left_inner_finger"]
    )

    assert len(merged) == len(merged_names)
    assert all(len(row) == len(merged_names) for row in merged)

    index = {name: i for i, name in enumerate(merged_names)}
    # 기존 허용 쌍 보존 — 여기가 깨지면 move_group 이 자기충돌로 계획을 못 한다.
    assert merged[index["link_a"]][index["link_b"]] is True
    assert merged[index["link_b"]][index["link_a"]] is True

    octomap = index["<octomap>"]
    for name in ("link_6", "rg2_left_inner_finger"):
        assert merged[octomap][index[name]] is True
        assert merged[index[name]][octomap] is True

    # 팔꿈치는 계속 voxel 을 피해야 한다.
    assert merged[octomap][index["link_a"]] is False

    # 두 번 적용해도(노드 재시작) 행이 늘어나지 않는다.
    again_names, again = motion.merge_octomap_acm(
        merged_names, merged, ["link_6", "rg2_left_inner_finger"]
    )
    assert again_names == merged_names
    assert again == merged


def test_mask_cloud_polygon_drops_projected_points_inside_mask():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    intrinsics = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 50.0}
    polygons_px = [[30, 30, 70, 30, 70, 70, 30, 70]]  # 픽셀 30~70 정사각형 1개

    xyz = np.array(
        [
            [0.0, 0.0, 1.0],   # (u,v)=(50,50) -> 폴리곤 안 -> 제거
            [0.5, 0.0, 1.0],   # (u,v)=(100,50) -> 이미지 밖(width=100) -> 보존
            [0.0, 0.0, 0.0],   # depth<=0, 투영 불가 -> 보존
            [np.nan, np.nan, np.nan],  # 무효 depth -> 보존
        ],
        dtype=np.float32,
    )

    keep = motion._mask_cloud_polygon(
        xyz, polygons_px, intrinsics, width=100, height=100
    )

    assert list(keep) == [False, True, True, True]


def test_mask_cloud_polygon_unions_multiple_instances_of_same_class():
    """같은 class_name 인스턴스가 여러 개면 마지막 것만 남기지 않고 전부 제외한다.

    _detection_callback 이 마지막 인스턴스로 덮어쓰면, 실제 pick 대상이 아닌
    다른 인스턴스의 마스크만 남아 정작 걸러야 할 물체는 그대로 샐 수 있다.
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    intrinsics = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 50.0}
    # 서로 겹치지 않는 두 정사각형: (10~20)과 (80~90).
    polygons_px = [
        [10, 10, 20, 10, 20, 20, 10, 20],
        [80, 80, 90, 80, 90, 90, 80, 90],
    ]

    xyz = np.array(
        [
            [-0.35, -0.35, 1.0],  # (u,v)=(15,15) -> 첫 인스턴스 안 -> 제거
            [0.35, 0.35, 1.0],    # (u,v)=(85,85) -> 두 번째 인스턴스 안 -> 제거
            [0.0, 0.0, 1.0],      # (u,v)=(50,50) -> 둘 다 밖 -> 보존
        ],
        dtype=np.float32,
    )

    keep = motion._mask_cloud_polygon(
        xyz, polygons_px, intrinsics, width=100, height=100
    )

    assert list(keep) == [False, False, True]
