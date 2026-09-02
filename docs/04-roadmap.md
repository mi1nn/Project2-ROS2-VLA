# 04. 개발 로드맵 — 10일 스프린트

관련 문서: [01 아키텍처](01-architecture.md) · [02 인터페이스 계약](02-interfaces.md) · [03 시스템 플로우](03-system-flow.md)

**기간 가정:** 2026-09-02 착수, **2026-09-11 완성 목표** (10일). 이 전제로 아래 일정을 짰다.

---

## 1. 일정표

| Day | 날짜 | 작업 | 완료 판정 |
| --- | --- | --- | --- |
| 1 | 09-02 (화) | 패키지 4개 스캐폴딩. **`kit_interfaces` srv/msg 확정 후 팀 공유**. `ros2 topic list \| grep dsr` 로 로봇 상태 토픽 확인 | `colcon build` 통과, `ros2 interface show` 로 6개 타입 확인 |
| 2 | 09-03 (수) | `ImgNode` 이식, `/detection/objects` **mock 발행** + `position_estimation` 골격 | `topic echo` + `service call` 왕복 성공 |
| 3 | 09-04 (목) | **hand-eye 캘리브레이션 재수행**, `T_gripper2camera.npy` 산출·검증 | `verify.py` 재투영 오차 확인 |
| 4 | 09-05 (금) | `motion.py`: `init` / `home` / `pick` / `place`, RG2 연동 | 손으로 입력한 좌표로 파지 성공 |
| 5 | 09-06 (토) | 좌표 파이프라인 결선. 실제 YOLO seg 모델 투입 | 1개 품목 자동 파지 성공 |
| 6 | 09-07 (일) | `controller`: **Component flatten + 실행 루프** + 상태머신 + `/kit/task_status` | 3 Component 연속 파지·배치 |
| 7 | 09-08 (월) | 음성/LLM 노드 결합, `/inspect_kit` 재검사 구현 | E2E 1회 관통 |
| 8 | 09-09 (화) | 재시도·실패 처리, `grasp_params.json` 품목별 튜닝 | 품목별 파지율 측정치 확보 |
| 9 | 09-10 (수) | E2E 20회 시험, 실패 로그 분석·개선 | 평가 결과표 |
| 10 | 09-11 (목) | 버퍼 / 문서 정리 / 시연 영상 | — |

---

## 2. 크리티컬 패스: Day 3 캘리브레이션

**여기가 밀리면 Day 4 이후가 전부 밀린다.** hand-eye 행렬 없이는 검출 좌표를 로봇 좌표로 바꿀 수 없고, 그러면 파지 자체가 성립하지 않는다.

캘리브레이션은 체커보드 자세를 여러 번 잡아 데이터를 모으는 작업이라 시간이 얼마나 걸릴지 예측이 어렵다. 게다가 데이터를 다 모은 뒤에야 품질을 알 수 있어서, 오차가 크면 처음부터 다시 해야 한다.

**대응: Day 1-2 와 병행해서 미리 시작한다.** 패키지 스캐폴딩은 로봇 없이 되는 작업이므로, 로봇을 쓸 수 있는 시간이 나면 캘리브레이션을 먼저 돌린다.

사용할 도구는 이미 있다.

```
reference/corecode/Calibration_Tutorial/
  data_recording.py        # 체커보드 자세별 데이터 수집
  handeye_calibration.py   # 행렬 산출
  verify.py                # 재투영 오차 확인 ← 이걸로 품질 판정
  T_gripper2camera.npy     # 이전 프로젝트 산출물 (참고용, 그대로 쓰면 안 됨)
```

레퍼런스의 `.npy` 는 **이전 하드웨어 구성의 값이다.** 카메라 마운트가 조금만 달라도 틀리므로 반드시 재생성한다. 급할 때 임시로 끼워 넣고 진행하되, 그 상태의 파지 결과는 신뢰하지 않는다.

---

## 3. 차단 의존성과 우회

| 의존 | 차단되는 작업 | 우회 |
| --- | --- | --- |
| 팀원의 YOLO seg 모델 | Day 5 실물 검출 | **Day 2 의 mock 발행자.** 고정 `DetectionArray` 를 1Hz 로 흘려 `position_estimation` 결선을 먼저 끝낸다 |
| 로봇 상태 토픽(posx) 부재 | position_estimation 의 hand-eye 입력 | **이미 우회됨.** controller 가 request 에 `robot_posx` 를 담는다 ([02 2.5절](02-interfaces.md)) |
| 팀원의 음성/LLM 노드 | Day 7 E2E | `command_json` 을 파일이나 `ros2 service call` 로 직접 주입해 Day 6 까지 진행 |
| 로봇 실물 점유 (팀 공유) | Day 3-5 | 좌표 변환·상태머신은 로봇 없이 self-check 로 검증. 로봇 시간은 캘리브레이션과 파지 튜닝에만 쓴다 |
| 키팅 트레이·품목 실물 | Day 8 튜닝 | 대체 물체로 파지 시퀀스만 먼저 검증 |

**우회의 핵심은 mock 이다.** 다른 팀원을 기다리며 노는 날이 하루도 없어야 10일에 맞는다. 계약([02 문서](02-interfaces.md))을 Day 1 에 못 박는 이유가 이것이다 — 계약이 있으면 상대 구현 없이도 내 쪽을 완성할 수 있다.

---

## 4. 하루 단위 상세

### Day 1 — 골격과 계약

```
src/kit_interfaces/{CMakeLists.txt, package.xml, msg/, srv/}
src/kit_vision/,  src/kit_voice/,  src/kit_robot/
```

`kit_interfaces` 는 `reference/cobot2/yolo_container/od_msg/` 구조를 베끼되, `std_msgs`/`builtin_interfaces` 의존 선언을 추가한다 (레퍼런스 `od_msg` 는 기본 타입만 써서 이게 없다). 나머지 세 패키지는 `ros2 pkg create --build-type ament_python` 로 만들고 빈 노드를 넣어 실행만 되게 한다 — `kit_robot` 은 노드 둘(`controller`, `position_estimation`)이다.

**끝나면 즉시 팀에 srv/msg 를 공유한다.** 이게 오늘의 진짜 산출물이다.

`ros2 topic list | grep dsr` 로 로봇 상태 토픽에 posx 가 있는지도 확인한다. 있으면 `position_estimation` 이 구독할 수 있고, 없으면 request 필드를 쓴다. **어느 쪽이든 계약은 안 바뀐다.**

### Day 2 — 비전 래퍼와 mock

`realsense.py` 의 `ImgNode` 를 이식한다(거의 수정 없음). `object_detection` 은 실제 추론 대신 고정된 `DetectionArray` 를 1Hz 로 발행하는 mock 으로 만든다. ROS 파라미터로 mock/real 을 전환하게 해두면 Day 5 에 모델만 갈아끼우면 된다.

**`position_estimation` 골격도 오늘 만든다.** 로봇 API 를 import 하지 않는 순수 계산 노드라 로봇 없이 완성할 수 있다 — mock 토픽 + `ros2 service call` 로 hand-eye 변환까지 검증 가능하다. 로봇을 못 쓰는 날에 가장 많이 진도를 뺄 수 있는 작업이다.

카메라가 실제로 붙어 있다면 intrinsic 수신과 depth 프레임 수신까지는 이날 확인해둔다.

### Day 3 — 캘리브레이션

위 2절 참조. 산출물 `T_gripper2camera.npy` 를 `kit_robot/resource/` 에 넣는다.

### Day 4 — 모션 기본기

`motion.py` + `onrobot.py`. `init()` → `home()` → 좌표를 손으로 입력해서 `pick()` / `place()` 가 도는지 본다. 이 단계에서 `place_slots.json` 의 슬롯 좌표를 실측해 채운다 (`reference/cobot2/rokey_cobot2/rokey_cobot2/basic/get_current_pos.py` 로 현재 자세를 읽어 기록).

**속도를 낮게 시작한다.** 레퍼런스 기본값이 `VELOCITY, ACC = 60, 60` 인데, 처음 좌표를 검증할 때는 더 낮춰서 이상하면 멈출 수 있게 한다.

### Day 5 — 첫 자동 파지

`position_estimation` 에 실제 검출을 물린다 (mock → real 전환). **이 프로젝트의 첫 진짜 마일스톤이다.** "YOLO 가 본 물체를 로봇이 집는다" 가 여기서 성립한다.

여기서 `z_offset` 초기값을 잡는다. 레퍼런스 `-35.0` 에서 시작해 실측으로 조정.

### Day 6 — 상태머신

`controller.py`. 레시피를 **Component 리스트로 flatten** 하고 하나씩 `execute_component()` 를 돌린다. 실패한 Component 를 건너뛰고 다음으로 넘어가는지가 이날의 핵심 검증이다. 음성 노드가 아직 없으면 `command_json` 을 직접 주입해서 시험한다.

### Day 7 — 통합

음성 노드 결합 + `/inspect_kit`. 검사 자세는 관찰 자세와 다를 수 있다(키팅 트레이를 봐야 함) — 별도 자세로 정의한다.

### Day 8 — 튜닝

`grasp_params.json` 을 품목별로 채운다. 품목당 10회씩 파지해서 성공률을 기록한다. 실패하는 품목은 폭·힘·`z_offset` 을 조정한다. **코드를 고치지 말고 JSON 만 고친다.** 그러라고 파일로 뺐다.

### Day 9 — 평가

E2E 20회. 기획서의 정량 평가 항목(파지 성공률, E2E 성공률, 연속 성공 횟수)을 채운다. 실패는 `/kit/task_status` 의 `detail` 로 남으므로 `ros2 bag record /kit/task_status` 를 켜두고 돌린다.

### Day 10 — 버퍼

일정은 밀린다. 이 날을 미리 비워두는 게 계획이다.

---

## 5. 위험 요소

| 위험 | 영향 | 완화 |
| --- | --- | --- |
| 캘리브레이션 오차 큼 | 파지가 계속 빗나감 | Day 3 에 `verify.py` 로 조기 판정. 오차 크면 즉시 재수행 |
| 로봇 실물 점유 경쟁 | 전 일정 지연 | 로봇 필요 작업(3,4,5,8)과 불필요 작업(1,2,6 일부)을 분리해 스케줄링 |
| YOLO 모델 지연 | Day 5 지연 | mock 으로 Day 6 까지 선행 |
| 품목별 파지 난이도 편차 | 특정 품목 성공률 저조 | Day 8 에 드러난다. 최악의 경우 해당 품목을 레시피에서 제외하고 사유를 기록 (기획서도 "구매·파지 가능성 확인 후 확정" 이라고 명시) |
| OpenAI 크레딧 소진 | 음성 경로 중단 | 에러코드로 즉시 식별되게 해둠. 잔액 사전 확인 |
