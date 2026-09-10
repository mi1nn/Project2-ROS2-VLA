# 06. Controller·GraspGenX 실행 가이드

관련 문서: [01 아키텍처](01-architecture.md) · [02 인터페이스](02-interfaces.md) · [03 플로우](03-system-flow.md) · [07 테스트 시나리오](07-test-scenario.md)

## 1. 현재 구현 범위

현재 브랜치에는 두 실행 경로가 있다.

1. **기존 Controller 경로:** 음성 명령 → YOLO 검출 → `position_estimation` 좌표 → `Motion.pick_component()` → 슬롯 배치 → 검사
2. **FoundationPose + GraspGenX 독립 시험:** 외부 NPZ/NPY 파일 → `grasp_pick_test.py` → `Motion.pick_graspgenx_candidates()`

`controller.py`의 `main()`은 실제 MoveIt2 기반 `Motion(node)`을 생성한다. 다만 현재 Controller가 Motion에 없는 `set_octomap_exclusion_component()`를 호출하고, 인자를 받지 않는 `move_to_inspection_pose()`에 `clear_before=True`를 전달한다. **이 두 호환성 문제를 정리하기 전에는 Controller E2E를 실행하지 않는다.** 이 문서는 현재 상태를 기록하며 코드를 수정하지 않는다.

| 파일 | 현재 역할 |
| --- | --- |
| `kit_robot/controller.py` | 7단계 상태머신, 서비스 client, Motion 호출, 결과 발행 |
| `kit_robot/controller_model.py` | ROS 없는 명령 검증, Component·Attempt, 슬롯 할당 |
| `kit_robot/motion.py` | MoveIt2 이동, TF, planning scene, OctoMap, RG2, 기존/GraspGenX 파지 |
| `kit_robot/position_estimation.py` | 검출 캐시, hand-eye 변환, 좌표·검사 서비스 |
| `kit_robot/grasp_pick_test.py` | 외부 GraspGenX 후보를 변환·미리보기·실행하는 독립 스크립트 |
| `resource/controller.yaml` | 지원 품목, 슬롯 이름, Controller 시간·시도 설정 |
| `config/motion.yaml` | MoveIt2, 자세, 슬롯 좌표, keepout, GraspGenX 설정 |
| `resource/grasp_params.json` | 품목별 그리퍼 폭·힘·접근 거리와 기존 좌표 보정값 |

위 경로는 `src/kit_robot/` 기준이다.

## 2. Controller 설정값

| 파라미터 | 코드 기본값 | 현재 YAML | 의미 |
| --- | ---: | ---: | --- |
| `supported_names` | 필수, 기본 목록 없음 | 한글 품목 9종 | `class_names.json`과 일치시켜 관리 |
| `slot_names` | 필수, 기본 목록 없음 | `slot_1`~`slot_6` | 공용 배치 순서 |
| `service_ready_timeout_sec` | 20초 | 20초 | 각 서비스 준비 대기 제한 |
| `command_timeout_sec` | 60초 | 60초 | 웨이크워드·STT·LLM을 포함한 명령 응답 제한 |
| `pose_timeout_sec` | 5초 | 15초 | 좌표 서비스 응답 제한 |
| `max_age_sec` | 1초 | 3초 | 좌표·검사에 허용할 검출 나이 |
| `observation_settle_sec` | 1.2초 | 4초 | 관찰 자세 이동 후 정착 시간 |
| `max_attempts` | 2회 | 3회 | 최초 시도를 포함한 Component 상한 |
| `inspect_timeout_sec` | 5초 | 10초 | 검사 서비스 응답 제한 |
| `inspection_settle_sec` | 1.2초 | 4초 | 검사 자세 이동 후 정착 시간 |
| `restart_delay_sec` | 5초 | 미지정 | 다음 작업을 받기 전 간격 |

Controller의 main loop는 `spin_once(..., timeout_sec=0.1)` 뒤 `timer_tick()`을 호출한다. Motion 이동은 완료될 때까지 동기적으로 기다리므로 이동 중 0.1초 상태 갱신 주기를 보장하지 않는다.

## 3. 공통 준비

각 터미널에서 ROS와 workspace 환경을 활성화한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to kit_robot
source install/setup.bash
```

MoveIt2/로봇 bringup과 RealSense를 먼저 시작한다. 최소한 다음 endpoint와 TF가 준비되어야 Motion 초기화가 완료된다.

```bash
ros2 action list | grep -E '/dsr01/(move_action|execute_trajectory)'
ros2 service list | grep -E '/dsr01/(compute_cartesian_path|apply_planning_scene|clear_octomap|get_planning_scene)'
ros2 topic echo /camera/depth/color/points --once
ros2 run tf2_ros tf2_echo base_link link_6
```

`motion.yaml`의 endpoint, `base_frame`, `eef_link`, 카메라 frame은 실제 bringup의 이름과 일치해야 한다. 로봇 주변을 비우고 비상 정지 장치를 사용할 수 있는 상태에서 저속으로 검증한다.

## 4. 기존 Controller 경로

### 4.1 실행 전 중단 조건

현재 브랜치는 아래 두 호출 계약이 맞지 않으므로 그대로 실행하지 않는다.

- `controller.py`: `set_octomap_exclusion_component(component.name)` 호출 — 현재 Motion에 메서드 없음
- `controller.py`: `move_to_inspection_pose(clear_before=True)` 호출 — 현재 Motion 메서드는 인자 없음

향후 코드 작업에서 계약을 정리한 뒤, `/get_command`, `/get_component_pose`, `/inspect_kit`, 비전 토픽, MoveIt/RG2가 모두 준비된 상태에서 실행한다.

```bash
ros2 run kit_robot controller --ros-args \
  --params-file src/kit_robot/resource/controller.yaml
```

결과는 Controller 실행 전에 별도 터미널에서 구독한다.

```bash
ros2 topic echo /kit/task_status
```

```bash
ros2 topic echo /kit/component_result
```

음성 노드는 서비스 요청을 받은 뒤 마이크를 열고 웨이크워드를 최대 30초 기다린다.

```bash
ros2 run kit_voice get_command
```

`kit_voice`가 사용하는 OpenAI 키를 문서나 로그에 복사하지 않는다. `/kit/command_result` publisher는 존재하지만 오디오 열기 실패, rate limit, 명령 검증 실패, 기타 LLM 오류 분기에서는 현재 메시지를 발행하지 않는다.

## 5. FoundationPose + GraspGenX 독립 시험

이 시험은 Controller, 음성, `GetComponentPose`, `InspectKit`을 사용하지 않는다. 대신 다음 외부 파일이 필요하다.

- 후보 NPZ: `grasp_poses` `(N,4,4)`와 `scores` `(N,)`
- 완성 객체 NPY: 유한한 XYZ 점이 10개 이상인 `(N,3)` 포인트클라우드

### 5.1 변환-only dry-run

```bash
python3 -m kit_robot.grasp_pick_test \
  --component 컵라면 \
  --candidate-file /absolute/path/to/object_safe_grasps.npz \
  --complete-object-pc /absolute/path/to/complete_object_pc.npy
```

`--execute`가 없으면 후보를 점수순으로 정렬하고 좌표 변환 결과만 출력하며 로봇·그리퍼 명령은 실행하지 않는다. 그러나 스크립트는 `Motion` 전체를 생성하므로 MoveIt 액션·서비스와 TF가 필요하고 RG2 연결도 시도한다.

`--move-observation`은 dry-run 옵션이 아니다. `--execute`가 없어도 실제 로봇을 `motion.yaml`의 관찰 자세로 이동시킨다.

### 5.2 실제 후보 실행

아래 조건을 모두 확인한 뒤에만 실행한다.

- 후보와 완성 객체 포인트클라우드가 같은 관찰 시점·카메라 좌표계에서 생성되었다.
- `camera_extrinsics.validated`, `grasp_to_tool.validated`, `execution_enabled`가 모두 `true`다.
- `camera_frame`, `tool_frame`, `eef_link`와 `grasp_to_tool.matrix`를 실제 장착 상태에서 검증했다.
- dry-run 목표를 RViz에서 확인했고 pregrasp/접근 경로와 OctoMap 제외 반경이 안전하다.

```bash
python3 -m kit_robot.grasp_pick_test \
  --component 컵라면 \
  --candidate-file /absolute/path/to/object_safe_grasps.npz \
  --complete-object-pc /absolute/path/to/complete_object_pc.npy \
  --max-candidates 10 \
  --move-observation \
  --execute
```

실행 시 Motion은 `T_base_camera @ T_camera_grasp @ T_grasp_tool @ T_tool_eef`로 목표를 만들고, 설정된 최대 개수까지 pregrasp → 직선 접근 → 그리퍼 닫기 → 후퇴를 시도한다. 완성 객체 포인트클라우드는 대상 주변 OctoMap 구형 제외 영역을 계산하는 데 사용한다.

현재 `motion.yaml`의 후보 파일 기본 경로는 특정 개발 PC의 절대 경로다. 다른 PC에서는 CLI 인자로 실제 경로를 반드시 지정한다. `setup.py`에는 `grasp_pick_test` console entry point가 없으므로 모듈 방식으로 실행한다.

## 6. 결과·재시작 정책

- `stale`/`not_detected`는 시도 상한 내 재관찰하고, `grasp_failed`는 복구 후 재관찰한다.
- `no_candidate`/`out_of_workspace`/시도 소진은 해당 Component를 실패 처리한다.
- TASK_FATAL 시 진행 중 Component는 FAILED, 미시작 Component는 SKIPPED로 기록한다.
- 실물 검사 PASS이고 TASK_FATAL·최종 복귀 실패가 없을 때 Task SUCCESS다.
- 명령 통신 실패, 크레딧 소진, 미지 명령 실패와 복구 실패는 자동 재시작을 차단한다.
- DB는 최초 저장된 SUCCESS Component만 재고에서 차감한다.

## 7. 현재 검증 한계

- `test_controller_model.py`는 순수 명령 모델만 검증하며 실제 Motion/서비스 E2E를 보장하지 않는다.
- `test_octomap_config.py`는 현재 제거된 `motion.yaml` 설정과 필터 함수를 기대해 최신 코드와 맞지 않는다.
- `setup.py`의 `motion_test` entry point는 존재하지 않는 `kit_robot.motion_test`를 가리킨다.
- GraspGenX는 독립 시험 경로이며 Controller·음성·DB 전체 흐름과 통합되지 않았다.
- Controller의 두 Motion 호출 불일치가 해소되기 전에는 실제 Controller 실행 결과를 검증할 수 없다.
- 검사 서비스는 트레이 ROI를 적용하지 않아 검사 화면에 원본 물체가 함께 보이면 오판할 수 있다.
- EMERGENCY, 일시 정지, 실행 중 강제 중단은 별도 프로토콜이 구현되어 있지 않다.

순수 모델 시험은 다음처럼 별도로 실행할 수 있다.

```bash
PYTHONPATH=src/kit_robot python3 -m pytest \
  src/kit_robot/test/test_controller_model.py -q
```
