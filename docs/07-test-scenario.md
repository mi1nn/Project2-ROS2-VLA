# 07. 전체 동작 테스트 시나리오

관련 문서: [03 시스템 플로우](03-system-flow.md) · [06 실행 가이드](06-controller-guide.md)

## 1. 목적과 현재 중단 조건

이 문서는 실제 MoveIt2·M0609·RG2와 현재 브랜치의 두 파지 경로를 검증하기 위한 절차다. 과거 `MotionDemo`·데모 서비스는 현재 저장소에 없으며 시험 대상으로 사용하지 않는다.

현재 Controller E2E에는 다음 호환성 문제가 있다.

- Controller가 현재 Motion에 없는 `set_octomap_exclusion_component()`를 호출한다.
- Controller가 `move_to_inspection_pose(clear_before=True)`를 호출하지만 현재 메서드는 인자를 받지 않는다.

따라서 **1~4절의 준비·독립 GraspGenX 시험은 진행할 수 있지만, 5절 Controller E2E는 두 계약을 정리한 뒤 진행한다.** 이 문서는 코드 수정 없이 중단 조건을 기록한다.

## 2. 공통 준비

각 터미널에서 같은 ROS domain과 workspace를 사용한다.

```bash
cd ~/Project2-ROS2-VLA
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

현재 브랜치와 변경 범위를 먼저 확인한다.

```bash
git branch --show-current
git status --short
```

## 3. MoveIt2·로봇·카메라 준비

실기 예시는 현재 저장소의 M0609 MoveIt launch 기준이다. IP와 RT 채널 값은 현장 설정을 확인한다.

```bash
ros2 launch dsr_moveit_config_m0609 start.launch.py \
  mode:=real \
  name:=dsr01 \
  model:=m0609 \
  gripper:=rg2 \
  host:=192.168.1.100 \
  gui:=true
```

RealSense 드라이버는 별도 터미널에서 시작한다. 아래 항목이 모두 보이지 않으면 Motion이나 GraspGenX 스크립트를 시작하지 않는다.

```bash
ros2 action list | grep -E '/dsr01/(move_action|execute_trajectory)'
ros2 service list | grep -E '/dsr01/(compute_cartesian_path|apply_planning_scene|clear_octomap|get_planning_scene)'
ros2 topic echo /camera/depth/color/points --once
ros2 run tf2_ros tf2_echo base_link link_6
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

기대 결과:

- 두 MoveIt action과 네 service가 `motion.yaml`의 이름으로 존재한다.
- 포인트클라우드의 `frame_id`와 TF tree가 연결된다.
- `base_link ← link_6`, `base_link ← camera_color_optical_frame` 변환이 연속으로 조회된다.

## 4. FoundationPose + GraspGenX 독립 시험

### 4.1 입력 파일 확인

후보 NPZ에는 `grasp_poses` `(N,4,4)`와 `scores` `(N,)`가 있어야 한다. 완성 객체 NPY는 유한한 XYZ 점이 10개 이상인 `(N,3)` 배열이어야 한다. 두 파일은 같은 관찰 시점과 카메라 좌표계를 기준으로 생성해야 한다.

로봇은 FoundationPose 입력을 촬영했을 때와 정확히 같은 관찰 자세에 있어야 한다. 현재 자세가 다르면 `T_base_camera`와 후보가 결합되는 기준이 달라진다.

### 4.2 변환-only dry-run

```bash
python3 -m kit_robot.grasp_pick_test \
  --component 컵라면 \
  --candidate-file /absolute/path/to/object_safe_grasps.npz \
  --complete-object-pc /absolute/path/to/complete_object_pc.npy \
  --max-candidates 10
```

통과 기준:

- 후보가 score 내림차순으로 출력된다.
- 모든 pregrasp/grasp pose가 유한한 6개 값이다.
- `T_base_camera`가 촬영 당시 자세와 일치한다.
- 로봇과 RG2에는 이동·개폐 명령이 전달되지 않는다.
- Motion 초기화 후 RViz planning scene에 반투명 빨간 keepout box와 OctoMap이 표시된다.

주의: dry-run도 Motion 전체를 초기화하므로 MoveIt endpoint·TF가 필요하고 RG2 연결을 시도한다. `--move-observation`을 붙이면 `--execute`가 없어도 실제 로봇이 움직이므로 이 단계에서는 사용하지 않는다.

### 4.3 실제 후보 실행

dry-run과 RViz 경로 확인, frame 보정 검증, 작업영역 정리가 모두 끝난 뒤 저속·비상 정지 준비 상태에서만 실행한다.

```bash
python3 -m kit_robot.grasp_pick_test \
  --component 컵라면 \
  --candidate-file /absolute/path/to/object_safe_grasps.npz \
  --complete-object-pc /absolute/path/to/complete_object_pc.npy \
  --max-candidates 10 \
  --move-observation \
  --execute
```

확인 항목:

- 관찰 자세에서 `T_base_camera`를 한 번 캡처한다.
- 완성 객체 포인트클라우드로 계산한 OctoMap 제외 중심·반지름이 대상만 감싼다.
- 후보마다 pregrasp → 직선 접근 → 그리퍼 닫기 → 역방향 후퇴 순서로 실행된다.
- 파지 검출 성공 시 다음 후보를 시도하지 않는다.
- 모든 후보 실패 또는 예외 후 OctoMap 제외 영역이 해제된다.

## 5. 기존 Controller E2E — 호환성 정리 후 실행

### 5.1 DB와 애플리케이션 노드

```bash
docker compose up -d postgres mongodb
docker compose ps postgres mongodb
```

DB 노드:

```bash
set -a
source .env
set +a
ros2 run kit_db db_node
```

카메라를 시작한 뒤 비전 컨테이너를 실행한다.

```bash
docker compose up -d vision
docker compose logs -f vision
```

각각 별도 터미널에서 좌표·음성 노드를 실행한다.

```bash
ros2 run kit_robot position_estimation
```

```bash
ros2 run kit_voice get_command
```

Controller 실행 전에 결과 토픽을 구독한다.

```bash
ros2 topic echo /kit/command_result
```

```bash
ros2 topic echo /kit/component_result
```

```bash
ros2 topic echo /kit/task_status
```

두 Motion 호출 계약이 정리된 것을 확인한 뒤 Controller를 시작한다.

```bash
ros2 run kit_robot controller --ros-args \
  --params-file src/kit_robot/resource/controller.yaml \
  -p restart_delay_sec:=60.0
```

### 5.2 E2E 통과 기준

| 단계 | 확인 내용 |
| --- | --- |
| 명령 | 동일 `task_id`로 서비스 응답과 `CommandResult`가 연결된다 |
| 검출·좌표 | 정착 이후 최신 검출을 사용하고 유효한 `target_pose`를 반환한다 |
| 실행 | Component별 pick/place 결과와 Attempt 수가 실제 동작과 일치한다 |
| 검사 | 원래 기대 수량과 실제 트레이 수량을 비교한다 |
| 종료 | 복귀 결과를 반영한 최종 TaskStatus가 한 번 발행된다 |
| DB | commands, component_executions, kit_executions가 같은 `task_id`로 조회된다 |
| 재고 | 최초 저장된 SUCCESS Component만 1개 차감된다 |

현재 음성 노드는 일부 실패 분기에서 `/kit/command_result`를 발행하지 않는다. 실패 시험에서 DB 기록이 없다면 먼저 서비스 응답과 음성 로그를 함께 확인한다.

## 6. DB 결과 확인

MongoDB 접속 후 실제 결과에서 복사한 task id를 한 번만 선언한다.

```javascript
const taskId = "TASK-실제_작업_ID";

db.commands.find({task_id: taskId}).pretty()
db.component_executions.find({task_id: taskId}).pretty()
db.kit_executions.find({task_id: taskId}).pretty()
```

PostgreSQL 재고 확인:

```sql
SELECT i.item_code, v.quantity, v.updated_at
FROM item AS i
JOIN inventory AS v USING (item_id)
ORDER BY i.item_id;
```

## 7. 알려진 시험 한계

- GraspGenX 독립 시험은 Controller·음성·검사·DB 통합을 검증하지 않는다.
- `test_octomap_config.py`는 현재 제거된 설정과 필터 함수를 기대하므로 현 상태의 회귀 시험으로 사용할 수 없다.
- `motion_test` console entry point는 존재하지 않는 모듈을 가리키며, `grasp_pick_test` entry point는 등록되어 있지 않다.
- 검사 서버에는 트레이 ROI가 없어 원본 물체가 화면에 함께 잡히면 수량 판정이 왜곡될 수 있다.
- 실시간 사람 감지·EMERGENCY·실행 중 강제 중단은 구현 범위 밖이다.
