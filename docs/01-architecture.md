# 01. 아키텍처 — 패키지 구조와 노드 구성

관련 문서: [00 기획 개요](00-project-overview.md) · [02 인터페이스 계약](02-interfaces.md) · [03 시스템 플로우](03-system-flow.md) · [04 로드맵](04-roadmap.md)

---

## 1. 패키지 구조

```
src/
  kit_interfaces/   # ament_cmake  — srv/msg 만. 팀 전체가 공유하는 계약
  kit_vision/       # ament_python — YOLO Segmentation 추론 + RealSense + 카메라 좌표 발행
  kit_voice/        # ament_python — STT + LLM + 명령 검증기
  kit_robot/        # ament_python — controller(상태머신) + position_estimation + M0609/RG2 모션
  kit_db/           # ament_python — MongoDB 실행 추적 + PostgreSQL 품목·재고 관리
```

빌드는 의존 순서상 `kit_interfaces` → 나머지 4개 순이다.

```bash
colcon build --symlink-install --packages-select kit_interfaces
colcon build --symlink-install --packages-select kit_vision kit_voice kit_robot kit_db
source install/setup.bash
```

---

## 2. 왜 역할별로 패키지를 나누는가

레퍼런스 `reference/cobot2/pick_and_place_voice/` 는 한 패키지 안에 `robot_control/`, `voice_processing/`, `object_detection/` 세 도메인을 모두 넣었다. 동작하는 코드지만 이번 프로젝트 조건에서는 그대로 따라가면 안 된다. 근거는 다섯 가지다.

### 2.1 의존성 오염

단일 패키지는 `package.xml` 과 `setup.py` 가 하나뿐이라, 세 도메인의 의존성이 전부 한 덩어리로 묶인다.

| 도메인 | 현재 코드가 사용하는 주요 의존성 |
| --- | --- |
| 비전 | `ultralytics`, `torch`, CUDA 런타임, `opencv`, `numpy`, `cv_bridge` |
| 음성 | `sounddevice`, `openai`, `langchain-openai`, `openwakeword`, `numpy`, `scipy` |
| 로봇 | `moveit_msgs`, `tf2_ros`, `sensor_msgs`, `scipy`, `numpy`, `pymodbus` |

로봇 제어만 실행할 때 비전의 `torch`를 설치할 필요가 없고, 마이크가 없는 로봇 PC에서 음성 입력 의존성을 로드할 필요도 없다. 현재 비전은 컨테이너에서, 음성·로봇 노드는 호스트에서 실행하는 구성을 사용한다.

### 2.2 Docker 실행환경을 역할별로 분리한다

현재 `compose.yaml`은 PostgreSQL, MongoDB와 GPU 비전 노드만 컨테이너로 실행한다. RealSense 드라이버, 음성 노드와 로봇 제어 노드는 호스트에서 실행한다.

패키지를 역할별로 나누면 비전의 CUDA·PyTorch 환경을 로봇·음성 환경에 강제로 설치하지 않아도 된다. 로봇 쪽은 M0609/RG2 통합 URDF, ros2_control, MoveIt2와 OctoMap을 사용한다.

### 2.3 팀 병렬 개발에서의 충돌

4인이 음성 / 비전 / ROS2 / DB 로 나뉘어 동시에 작업한다. 단일 패키지면 `setup.py` 의 `entry_points`, `data_files`, `resource/` 디렉터리를 네 명이 매일 같이 건드린다. 이 파일들은 구조상 머지 충돌이 잦고, 충돌 해결을 잘못하면 남의 노드가 조용히 실행 불가가 된다.

패키지 경계를 소유권 경계와 일치시키면 각자 자기 `setup.py` 만 만진다. 충돌 지점은 `kit_interfaces` 하나로 좁혀지고, 그건 애초에 다 같이 합의해야 하는 파일이라 충돌이 드러나는 게 오히려 정상이다.

### 2.4 빌드 시간

`colcon build --packages-select kit_robot` 으로 로봇 코드만 재빌드한다. 하루에 수십 번 도는 사이클에서 이 차이가 누적된다. 단일 패키지면 그리퍼 상수 하나 고쳐도 전체가 다시 빌드된다.

### 2.5 계약이 강제된다

같은 패키지 안에 있으면 `from voice_processing.get_keyword import extract_keyword` 처럼 남의 내부 함수를 직접 import 하는 지름길이 열린다. 편해 보이지만 그 순간 팀원의 리팩터링이 내 노드를 깨뜨리는 구조가 된다.

패키지를 나누고 통신을 `kit_interfaces` 의 srv/msg 로만 하면, 팀 간 소통이 "srv 파일 리뷰" 로 환원된다. 인터페이스가 문서이자 계약이자 테스트 지점이 된다. 이게 분리의 가장 큰 실익이다.

### 2.6 감수하는 비용

공짜는 아니다. 패키지가 넷이면 터미널을 넷 띄우거나 launch 파일이 필요하고, `kit_interfaces` 를 고칠 때마다 전체 재빌드가 걸린다.

- 실행 복잡도 → 현재는 MoveIt/로봇, 비전, 음성, 좌표 추정, Controller와 DB를 각각 실행한다. 전체 실행 순서는 [07 테스트 시나리오](07-test-scenario.md)를 따른다.
- 인터페이스 변경 비용 → 오히려 장점으로 쓴다. 계약 변경은 비싸야 신중해진다. Day 1 에 srv/msg 를 확정하고 팀에 공유하는 이유다([04 로드맵](04-roadmap.md)).

---

## 3. 노드 구성

| 패키지 | 노드 | 역할 | 담당 |
| --- | --- | --- | --- |
| `kit_vision` | `object_detection` | YOLO seg 추론 + depth 결합 + 역투영. **카메라 좌표를 토픽 발행** | 래퍼는 나, 모델 학습은 팀원 |
| `kit_voice` | `get_command` (`get_command_node`) | 웨이크워드 → STT → LLM → JSON 검증. `/get_command` **서비스 서버** | 팀원 |
| `kit_robot` | `position_estimation` | 검출 토픽 구독 → hand-eye 변환 → **파지 자세 서비스 응답** | 나 |
| `kit_robot` | `controller` | component 단위 실행, 상태머신, `motion.py` 사용 | 나 |
| `kit_db` | `db_node` | `/kit/command_result`, `/kit/task_status`, `/kit/component_result` 구독. MongoDB 실행 추적 및 PostgreSQL 재고 관리 | 팀원 전담 |

담당 경계 요약: **오케스트레이션 · 로봇 제어 · 좌표 추정 · 인터페이스 계약 · 비전 노드 ROS2 래퍼가 내 몫이고, YOLO 모델 학습 · 음성/LLM · DB 적재는 팀원 몫이다.**

```
                      /camera/color/image_raw
 RealSense D435 ────▶ /camera/aligned_depth_to_color/image_raw ────▶ object_detection
                      /camera/color/camera_info                            │
                                                                           │ 토픽 (상시 발행)
                                           /detection/objects  ← 카메라 좌표 │
                                                                           ▼
 [사용자 음성] ──▶ get_command                                     position_estimation
                        │  │                                             ▲
                        │  └── /kit/command_result ──▶ db_node           │ srv /get_component_pose
                        │           CommandResult                        │  req: component, robot_posx, max_age_sec
                        │ srv /get_command                               │  res: target_pose + 원본 검출 정보
                        ▼                                                │
                    controller ──────────────────────────────────────────┘
                        │
                        ├── 연결 ────▶ Motion 객체 (MoveIt2 + OctoMap + M0609/RG2)
                        │
                        ├── /kit/task_status ────────────▶ db_node (TaskStatus)
                        └── /kit/component_result ───────▶ db_node (ComponentResult)
```

> **DB 적재 구조와 저장 정책은 [05 데이터베이스](05-database.md)를 기준으로 삼는다.** `get_command` 노드가 발행한 `CommandResult`는 `commands`, `controller`가 발행한 `TaskStatus`와 `ComponentResult`는 각각 `kit_executions`와 `component_executions`에 저장된다. 최초 저장된 `SUCCESS` Component만 PostgreSQL 재고를 1개 차감한다.

FoundationPose + GraspGenX 경로는 위 Controller 시퀀스에 아직 연결되지 않았다. 현재는 외부에서 생성한 `grasp_poses`·`scores` NPZ와 완성 객체 포인트클라우드 NPY를 `grasp_pick_test.py`가 읽어 `Motion.pick_graspgenx_candidates()`를 직접 호출하는 독립 시험 경로다.

**검출은 서비스가 아니라 토픽이다.** `object_detection` 은 요청과 무관하게 계속 돌면서 검출을 발행한다. 덕분에 `ros2 topic echo /detection/objects` 로 인식 상태를 언제든 볼 수 있고, 로봇을 세워둔 채 비전만 디버깅할 수 있다. 반면 좌표는 **요청 시점에 확정되어야** 하므로 서비스다 — 이 하이브리드가 이 시스템의 통신 구조다.

---

## 4. controller 와 motion 의 관계

### 4.1 Controller가 상태머신과 Motion 수명을 소유한다

`controller.py`의 `main()`은 먼저 Controller 노드를 만든 뒤 `Motion(node)`을 생성해 연결한다. Controller는 명령·좌표·검사 서비스를 호출하고 상태 전이를 관리하며, 실제 이동은 Motion 메서드에 위임한다.

Motion은 내부적으로 별도 ROS 노드와 `MultiThreadedExecutor`를 소유한다. MoveIt2의 MoveGroup·ExecuteTrajectory 액션, Cartesian path·planning scene 서비스, TF 조회, OctoMap 입력 클라우드 중계와 RG2 Modbus 제어를 한 객체에서 관리한다. Controller가 Motion의 내부 ROS 통신을 직접 다루지는 않는다.

### 4.2 그런데 position_estimation 은 왜 별도 노드여도 되나

좌표 계산은 서비스 경계로 분리되어 있다. Controller가 Motion의 TF 조회로 얻은 현재 로봇 자세(`robot_posx`)를 request에 담아 보내므로, `position_estimation`은 로봇 이동 API를 호출하지 않는 **순수 계산 노드**다. 근거와 데이터 계약은 [02 인터페이스](02-interfaces.md) 2.5절에 있다.

부수 효과가 크다 — 로봇 없이 단위 검증이 가능하고, 검출 토픽에 mock 을 물려 좌표 변환만 따로 시험할 수 있다.

### 4.3 현재 파지 경로는 둘로 나뉜다

- 기존 Controller 경로: YOLO 검출과 `position_estimation`의 `target_pose`를 받아 `Motion.pick_component()`로 집고, `motion.yaml`의 슬롯 좌표로 배치한다.
- FoundationPose + GraspGenX 경로: `grasp_pick_test.py`가 외부 NPZ·NPY 파일을 읽어 `Motion.pick_graspgenx_candidates()`를 직접 호출한다. 아직 Controller 상태머신이나 `GetComponentPose` 서비스와 연결되지 않았다.

현재 `feature/grasp` 브랜치에는 Controller가 삭제된 OctoMap 메서드를 호출하고, `move_to_inspection_pose()`에 현재 시그니처에 없는 인자를 전달하는 호환성 문제가 남아 있다. 따라서 문서에서 두 경로를 구분하며, 이 계약을 정리하기 전에는 Controller 전체 흐름이 실행 가능하다고 간주하지 않는다.

### 4.4 파일 배치

```
kit_robot/kit_robot/
  controller.py            # 노드. Enum + timer + 비동기 서비스, 실행·검사·결과 발행
  controller_model.py      # ROS 없는 명령 검증, Component·Attempt, 공용 슬롯 할당
  position_estimation.py   # 노드. 검출 구독 + hand-eye 변환 + 서비스 서버
  motion.py                # MoveIt2 이동, OctoMap 중계·제외, 기존/GraspGenX 파지
  grasp_pick_test.py       # GraspGenX 후보 파일을 사용하는 독립 시험 스크립트
  onrobot.py               # 모듈. RG2 Modbus 제어
  handeye_calibration.py   # hand-eye 캘리브레이션 유틸리티

kit_vision/kit_vision/
  object_detection.py      # 노드. YOLO seg + depth + 역투영 → 토픽 발행
  realsense.py             # 모듈. RealSense 영상·depth·camera_info 캐시
  yolo_model.py            # 모듈. YOLO segmentation 단일 프레임 추론

kit_db/kit_db/
  db_node.py               # 노드. 세 결과 토픽 구독 + mapper 검증 + 저장소 호출
  config.py                # 모듈. 환경 변수에서 데이터베이스 연결 설정을 로드
  message_mapper.py        # 모듈. ROS 메시지를 MongoDB에 저장 가능한 Python dictionary로 변환.
  mongodb.py               # DB 노드에서 사용되는 MongoDB 접속을 관리
  persistence.py           # message_mapper를 통해 ROS 메시지를 변환하는 기능을 제공하는 모듈
  postgres.py              # PostgreSQL DB와 상호작용하는 모듈
```

`controller_model.py`는 ROS 없이 명령 검증과 슬롯 할당을 단위 시험한다. 실제 장비 실행과 독립 GraspGenX 시험 방법은 [06 Controller 가이드](06-controller-guide.md)를 따른다. 좌표 변환에는 self-check를 붙인다([03 플로우](03-system-flow.md) 4.3절).

---

## 5. 자원 파일 배치

| 경로 | 내용 | 출처 |
| --- | --- | --- |
| `kit_robot/resource/T_gripper2camera.npy` | hand-eye 변환 행렬 (4×4) | `reference/corecode/Calibration_Tutorial/` 로 재생성 |
| `kit_robot/resource/grasp_params.json` | **클래스별 캘리브레이션 노브** | 실측으로 채운다 |
| `kit_robot/config/motion.yaml` | MoveIt2·자세·배치 슬롯·GraspGenX 실행 설정 | 로봇/작업대 실측과 검증 |
| `kit_vision/resource/*.pt` | YOLO seg 가중치 | 팀원 학습 산출물 |
| `kit_vision/resource/class_names.json` | 클래스 id ↔ 이름 | 팀원 |
| 외부 `*_safe_grasps.npz` | GraspGenX 후보 `grasp_poses`, `scores` | FoundationPose/GraspGenX 파이프라인 |
| 외부 `complete_object_pc.npy` | 객체 전체 포인트클라우드 | FoundationPose/GraspGenX 파이프라인 |

`grasp_params.json` 을 상수가 아니라 파일로 빼는 이유: 후보 품목 9종(컵라면, 햄, 수세미, 마스크, 일회용 숟가락, 여행용 티슈, 양갱, 샴푸, 분유)은 크기와 강성이 제각각이라 그리퍼 폭·힘·접근 높이를 하나의 상수로 덮을 수 없다. 레퍼런스의 `DEPTH_OFFSET = -35.0` 은 특정 공구 세트에 맞춰진 값이고, 우리 품목에서는 품목별로 다시 재야 한다.

```json
{
  "_default": { "strategy": "top_down", "width": 500, "force": 200, "z_offset": -35.0, "approach": 100.0 },
  "마스크":    { "strategy": "top_down", "width": 1100, "force": 200, "z_offset": -15.0, "approach": 80.0 }
}
```

값은 Day 8 에 품목별 반복 파지로 채운다. 코드 수정 없이 JSON 만 고쳐 재시험할 수 있어야 튜닝 사이클이 짧아진다.

---

## 6. 레퍼런스 코드 재사용 지도

새로 쓰는 코드는 최소로 한다. 검증된 레퍼런스에서 가져올 것들:

| 가져올 것 | 원본 경로 | 이식 위치 |
| --- | --- | --- |
| hand-eye 좌표 변환 (`transform_to_base`, `get_robot_pose_matrix`) | `reference/cobot2/pick_and_place_voice/robot_control/robot_control.py` | `kit_robot/position_estimation.py` |
| RG2 modbus 제어 (가상 그리퍼 fallback 포함) | `.../robot_control/onrobot.py` | `kit_robot/onrobot.py` (거의 그대로) |
| RealSense 구독 노드 `ImgNode` | `.../object_detection/realsense.py` | `kit_vision/realsense.py` (거의 그대로) |
| 픽셀 → 카메라 좌표 역투영, depth median | `.../object_detection/detection.py` | `kit_vision/object_detection.py` |
| YOLO 다중 프레임 집계 + IoU 병합 | `.../object_detection/yolo.py` | `kit_vision/yolo_model.py` (seg 대응 확장) |
| pick & place 모션 골격 | `.../robot_control/robot_control.py` | `kit_robot/motion.py` |
| LLM 프롬프트 구조 · 에러코드 반환 패턴 | `.../voice_processing/get_keyword.py` | `kit_voice/` (팀원) |
| 체커보드 hand-eye 캘리브레이션 | `reference/corecode/Calibration_Tutorial/handeye_calibration.py`, `verify.py` | 그대로 실행해서 `.npy` 산출 |
