"""Octomap 설정 두 파일이 서로 맞는지 검사한다.

topic 이 한 글자만 틀려도 move_group 은 조용히 빈 octomap 을 들고 돈다(에러 없음,
RViz 에 voxel 만 안 보임). 로봇을 띄우기 전에 잡는다.

실행: python3 -m pytest src/kit_robot/test/test_octomap_config.py
"""
import time
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


def test_remove_outlier_points_drops_isolated_points_keeps_dense_cluster():
    """대충 rung 흘려보낸 한 점(flying pixel)은 버리고, 같은 자리에 몰린
    "진짜 표면" 점들은 남긴다. voxel/octomap 부분삭제가 없으니, 이 단계에서
    안 걸러지면 그 점 하나가 영구 장애물이 된다.
    """
    np = pytest.importorskip("numpy")
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    # voxel_size=0.01, min_neighbors=4 기준.
    # 같은 voxel(0,0,0)에 몰린 점 5개(밀집 표면) + 멀리 떨어진 고립점 1개(플라잉 픽셀).
    dense_cluster = [[0.001 * i, 0.0, 1.0] for i in range(5)]
    stray = [[5.0, 5.0, 1.0]]
    invalid = [[np.nan, 0.0, 1.0], [0.0, 0.0, -1.0]]  # 무효 depth -> 무조건 버림

    xyz = np.array(dense_cluster + stray + invalid, dtype=np.float32)

    keep = motion._remove_outlier_points(xyz, voxel_size=0.01, min_neighbors=4)

    assert list(keep) == [True, True, True, True, True, False, False, False]


def test_flatten_floor_points_snaps_wobble_keeps_tall_object():
    """그레이징 앵글 계단(바닥 높이 흔들림)은 모두 같은 높이로 눌리고,
    band_m 보다 훨씬 위에 있는 물체는 그대로 남는다.
    """
    np = pytest.importorskip("numpy")
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    # 바닥: 0.100~0.110m 사이에서 계단처럼 흔들리는 점 20개.
    floor = [[float(i), 0.0, 0.10 + 0.005 * (i % 3)] for i in range(20)]
    # 물체: 바닥보다 8cm 위 (band_m=0.015 를 훨씬 벗어남) -> 건드리지 않아야 함.
    tall_object = [[0.0, 1.0, 0.18]]

    xyz = np.array(floor + tall_object, dtype=np.float64)

    out = motion._flatten_floor_points(xyz, band_m=0.015, floor_percentile=5.0)

    assert len(np.unique(out[:20, 2])) == 1, "바닥 흔들림이 한 높이로 안 눌렸다"
    assert out[20, 2] == xyz[20, 2], "band_m 밖 물체가 바닥으로 눌려버렸다"


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


def test_mask_cloud_polygon_padding_drops_depth_noise_at_object_edge():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    intrinsics = {"fx": 100.0, "fy": 100.0, "ppx": 50.0, "ppy": 50.0}
    polygon = [[45, 45, 55, 45, 55, 55, 45, 55]]
    xyz = np.array(
        [
            [0.0, 0.0, 1.0],   # (50, 50): target mask
            [0.08, 0.0, 1.0],  # (58, 50): boundary flying pixel
            [0.2, 0.0, 1.0],   # (70, 50): real nearby geometry
        ],
        dtype=np.float32,
    )

    keep = motion._mask_cloud_polygon(
        xyz, [polygon], intrinsics, width=100, height=100, padding_px=10
    )

    assert list(keep) == [False, False, True]


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


def test_wait_for_exclusion_mask_returns_immediately_once_mask_present():
    """마스크가 이미 와있으면 기다리지 않고 바로 반환 — 게이트가 늦게 열리면
    settle 시간을 그만큼 잡아먹으므로, 준비돼 있을 때 지연이 없어야 한다.
    """
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    class Fake:
        _octomap_exclusion_component = "양갱"
        _latest_detection_masks = {"양갱": [[0, 0, 1, 0, 1, 1]]}
        logger = type("L", (), {"warn": lambda self, *a, **k: None})()

    started = time.monotonic()
    motion.Motion._wait_for_exclusion_mask(Fake(), timeout_sec=1.0)
    assert time.monotonic() - started < 0.2


def test_exclusion_mask_must_be_newer_than_pick_request():
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    class Fake:
        _octomap_exclusion_component = "양갱"
        _octomap_exclusion_min_stamp_ns = 200
        _latest_detection_stamp_ns = 199
        _latest_detection_masks = {"양갱": [[0, 0, 1, 0, 1, 1]]}

    assert not motion.Motion._has_fresh_exclusion_mask(Fake())
    Fake._latest_detection_stamp_ns = 200
    assert motion.Motion._has_fresh_exclusion_mask(Fake())


def test_wait_for_exclusion_mask_times_out_and_opens_anyway():
    """감지가 끝내 안 오면(예: 이 컴포넌트가 화면에서 사라짐) 무한정 막지
    않고 timeout 뒤에 그냥 게이트를 연다 — pick 자체는 곧 no_candidate 로
    실패해서 다른 경로로 처리된다.
    """
    motion = pytest.importorskip(
        "kit_robot.motion", reason="ROS 런타임 없음 (로봇/도커에서 실행)"
    )

    class Fake:
        _octomap_exclusion_component = "양갱"
        _latest_detection_masks = {}
        logger = type("L", (), {"warn": lambda self, *a, **k: None})()

    started = time.monotonic()
    motion.Motion._wait_for_exclusion_mask(Fake(), timeout_sec=0.15)
    elapsed = time.monotonic() - started
    assert 0.1 <= elapsed < 1.0
