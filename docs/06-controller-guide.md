# 06. Controller 실행·검증 가이드

관련 문서: [01 아키텍처](01-architecture.md) · [02 인터페이스](02-interfaces.md) · [03 플로우](03-system-flow.md) · [04 로드맵](04-roadmap.md)

## 1. 현재 구현 범위

Controller는 명령 수신 → 검증·슬롯 배정 → 관찰·좌표 요청 → 파지·배치 → 검사 → 복귀·최종 발행을 처리한다.
현재 `main()`은 `Controller(motion=MotionDemo())`로 실행한다. 실제 로봇 동작은 연결하지 않았다.

| 파일 | 역할 |
| --- | --- |
| `kit_robot/controller.py` | 상태 전이, 서비스 client, Motion 호출, 재시도·종료, 토픽 발행 |
| `kit_robot/controller_model.py` | ROS 없는 명령 검증, Component·Attempt, 슬롯 할당·기대 수량 |
| `kit_robot/motion_demo.py` | Motion 메서드 7개의 호출 로그와 성공 반환 |
| `kit_robot/controller_demo_services.py` | 명령·좌표·검사 서비스의 성공 응답 |
| `resource/controller.yaml` | 지원 품목, 공용 슬롯, 서비스 대기·정착 파라미터 |
| `test/test_controller_model.py` | 모델 단위 테스트 7개 |

위 경로는 `src/kit_robot/` 기준이다. 클래스·메서드 설명과 파라미터 선정 기준은 controller.py의 docstring·주석에도 정리되어 있다.

## 2. 설정값

ROS 파라미터는 YAML 또는 CLI로 덮어쓸 수 있다. YAML은 resource 설치 규칙에 포함되어 있다.

| 파라미터 | 코드 기본값 | 제공 YAML | 의미 |
| --- | --- | --- | --- |
| `supported_names` | 기본 목록 없음, 필수 | 한글 품목 9종 | class_names.json과 수동으로 일치시켜 관리 |
| `slot_names` | 기본 목록 없음, 필수 | slot_1~slot_6 | 품목별 전용 구분 없는 공용 배치 순서 |
| `service_ready_timeout_sec` | 20초 | 20초 | 각 서비스 준비 대기 제한 |
| `command_timeout_sec` | 60초 | 60초 | 명령 요청부터 음성·STT·LLM 응답까지 |
| `pose_timeout_sec` | 5초 | 10초 | 좌표 요청 후 응답 제한 |
| `max_age_sec` | 1초 | 1초 | 좌표·검사 검출의 허용 나이 |
| `observation_settle_sec` | 1.2초 | 1.2초 | 관찰 이동 완료 후 정착 |
| `max_attempts` | 2회 | 미지정, 코드 기본값 | 최초 시도를 포함한 Component 시도 상한 |
| `inspect_timeout_sec` | 5초 | 5초 | 검사 요청 후 응답 제한 |
| `inspection_settle_sec` | 1.2초 | 1.2초 | 검사 이동 완료 후 정착 |
| `restart_delay_sec` | 5초 | 미지정, 코드 기본값 | 최종 발행 후 다음 작업까지의 간격 |

timer 주기는 코드에 0.1초로 고정되어 있다. 동기 Motion 호출 중 같은 주기의 tick이 보장되지는 않는다.
timeout은 실측 전 설정이다. 정착 시간은 max_age_sec보다 길어야 하며 시계 일치·촬영 지연도 실제 환경에서 확인한다.

중복 품목은 최초 등장 순서로 합산한다. 예를 들어 컵라면·마스크·컵라면 각 1개는
컵라면 2개, 마스크 1개 순서로 slot_1~slot_3에 배정한다. 빈 슬롯명·중복 슬롯명·슬롯 수 초과는 검증에서 거부한다.

## 3. 로봇 없는 데모 실행

아래 명령은 저장소 루트 기준이다. 각 터미널에서 ROS 환경과 workspace 환경을 활성화한다.
실제 command·position_estimation 서버는 함께 실행하지 않는다. 같은 이름의 서비스를 중복 제공하면 응답 서버를 특정할 수 없다.

### 3.1 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to kit_robot
source install/setup.bash
```

### 3.2 결과 구독 — Controller 실행 전에 시작

각각 별도 터미널에서 환경을 활성화한 뒤 실행한다. 기본 토픽은 과거 결과를 재생하지 않으므로 먼저 구독한다.

```bash
ros2 topic echo /kit/task_status
```

```bash
ros2 topic echo /kit/component_result
```

### 3.3 데모 서비스

별도 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/kit_robot/kit_robot/controller_demo_services.py
```

컵라면 1개 명령, 가상 자세 `[0.0] * 6`, 기대 수량과 같은 검사 수량·PASS를 반환한다.
실제 검출·검사를 수행하지 않으므로 이 서비스의 좌표를 실제 Motion에 전달하지 않는다.

### 3.4 Controller

별도 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run kit_robot controller --ros-args \
  --params-file src/kit_robot/resource/controller.yaml
```

설치된 YAML을 사용하려면 params-file 경로를
`"$(ros2 pkg prefix kit_robot)/share/kit_robot/resource/controller.yaml"`로 바꾼다.

기대 흐름은 `IDLE → LISTEN → VALIDATE → OBSERVE → EXECUTE → INSPECT → REPORT`다.
컵라면의 ComponentResult는 SUCCESS, attempt_count=1, slot=slot_1이며 최종 TaskStatus는 SUCCESS와 검사 PASS다.
최종 Task 발행 전에 move_home 로그가 출력된다. 기본 5초 후 새 task_id로 반복한다.
데모 서버는 매 요청마다 같은 명령을 반환하므로 한 번만 확인하려면 Controller를 직접 종료한다.

## 4. 실제 음성 노드 연결 시

데모 서버 하나가 서비스 세 개를 제공하므로 실제 음성 노드와 함께 켜지 않는다.
실제 음성과 가상 비전을 혼합하려면 서비스가 중복되지 않도록 별도 테스트 구성이 필요하다.

음성 노드는 `src/kit_voice/resource/.env`의 `OPENAI_API_KEY`를 설치 경로에서 읽는다.
파일 생성 후 kit_voice를 재빌드하고 환경을 활성화한다. 키 내용은 문서·로그에 복사하지 않는다.

```bash
colcon build --symlink-install --packages-select kit_voice
source install/setup.bash
ros2 run kit_voice get_command
```

음성 노드는 서비스 요청을 받아야 마이크를 열고 웨이크워드를 최대 30초 기다린다.
응답 후에는 다음 요청을 기다린다. 키팅 중 스스로 웨이크워드를 다시 기다리거나
키팅 시간이 명령 timeout에 누적되지는 않는다. 실제 음성 노드의 응답 JSON에 있는
raw_text/task_id는 Controller 검증 결과에서 제외하며 원래 task_id를 유지한다.

## 5. 결과·재시작 정책

- stale/not_detected는 시도 상한 내 재관찰, grasp_failed는 안전 복구 후 재관찰한다.
- no_candidate/out_of_workspace/시도 소진은 해당 Component만 실패 처리한다.
- TASK_FATAL 시 미종료 시도는 FAILED, 미시작 Component는 SKIPPED로 기록한다.
- 실물 검사 PASS이고 TASK_FATAL·최종 복귀 실패가 없을 때 Task SUCCESS다. Component 실패 이력은 보존한다.
- 검사 불일치는 FAIL, 유효한 검사 데이터가 없거나 검사 통신에 실패하면 ERROR다.
- IDLE 전환에서는 이전 작업의 RUNNING을 다시 발행하지 않는다. Component 발행 index는 작업마다 초기화한다.
- 명령 통신 실패·크레딧 소진·미지 명령 실패와 복구 실패는 REPORT에서 자동 재시작을 차단한다.
- 현재 좌표·검사·배치 실패는 최종 복귀가 성공하면 재시작 가능하다. TASK_FATAL은 항상 재시작 금지를 의미하지 않는다.

DB는 기존대로 최초 SUCCESS Component만 재고에서 차감한다. Task PASS로 FAILED Component의 재고를 소급 변경하지 않는다.

## 6. 검증 결과와 후속 작업

모델 단위 테스트:

```bash
PYTHONPATH=src/kit_robot python3 -m pytest \
  src/kit_robot/test/test_controller_model.py -q
```

PR 작성 시 위 테스트 7개 통과를 확인했다. 개발 중 명령 수신·검증과 데모 실행은 수동 확인했으며,
실제 로봇·전체 DB 연동 및 모든 오류 경로의 통합 검증 완료를 의미하지 않는다.

후속 검증·구현 항목:

- 실제 Motion과 슬롯 좌표 연결, DSR 초기화·executor 응답 처리 확인.
- 테스트 서비스의 실패 응답·지연과 Motion 파지·복구 실패를 주입해 재시도·SKIPPED·중복 발행 검증.
- 검사 화면에서 원본 물체가 제외되는지 확인. 현재 검사 서버에는 ROI 필터가 없다.
- 다음 작업 전 트레이 교체·빈 상태 확인. 현재 자동 재시작은 이를 기다리지 않는다.
- timer 전체의 예외 처리와 직렬화·발행 오류 대응 검토. 모든 예외가 REPORT로 전환되지는 않는다.
- 음성 is_wakeup/close의 미처리 예외와 명령 전체 응답 시간 검증.
- EMERGENCY·일시 정지·실행 중 강제 중단은 별도 요구사항 확정 후 구현.
