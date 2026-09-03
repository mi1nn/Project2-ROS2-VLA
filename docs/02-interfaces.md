# 02. 인터페이스 계약 (`kit_interfaces`)

관련 문서: [01 아키텍처](01-architecture.md) · [03 시스템 플로우](03-system-flow.md)

이 문서가 팀 간 유일한 공식 계약이다. 노드 내부 구현은 각자 자유지만, 여기 정의된 srv/msg 를 우회해서 남의 패키지 코드를 직접 import 하지 않는다.

**변경 절차:** srv/msg 수정은 전원 재빌드를 유발한다. Day 1 에 확정하고, 이후 변경은 팀 전체 합의 후 이 문서를 먼저 고친 다음 코드를 고친다.

---

## 0. 단위·좌표계 규약 — 최우선

> 팀 간 버그의 최대 원인이다. 예외 없이 아래를 따른다.

| 항목 | 규약 |
| --- | --- |
| 길이 | **mm** (전 구간. m 는 어디에도 쓰지 않는다) |
| 각도 | **도(degree)**, ZYZ 오일러 |
| 로봇 posx | `[x, y, z, rx, ry, rz]` — mm, deg, 베이스 좌표계 |
| RealSense depth | 원본이 uint16 mm 단위. **변환하지 않고 그대로 쓴다** |
| 카메라 좌표계 | 광축 +Z 전방, +X 우측, +Y 하방 (OpenCV 관례) |
| 인덱스 | 이미지 픽셀은 `(x=열, y=행)` |

RealSense 원본이 mm 이고 두산 posx 도 mm 이므로, **전 구간 mm 을 유지하면 단위 변환 코드가 아예 필요 없다.** 레퍼런스도 이 전제로 동작한다.

---

## 1. 패키지 구성

`kit_interfaces` 는 `ament_cmake` 패키지다. `ament_python` 으로는 IDL 생성이 안 된다.

```
kit_interfaces/
  CMakeLists.txt
  package.xml
  msg/
    DetectedObject.msg
    DetectionArray.msg
    TaskStatus.msg
  srv/
    GetComponentPose.srv
    InspectKit.srv
    GetCommand.srv
```

`CMakeLists.txt` 는 레퍼런스 `reference/cobot2/yolo_container/od_msg/CMakeLists.txt` 를 그대로 따르되 파일 목록만 교체한다.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectionArray.msg"
  "msg/TaskStatus.msg"
  "srv/GetComponentPose.srv"
  "srv/InspectKit.srv"
  "srv/GetCommand.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)
```

`DetectionArray.msg` 가 `std_msgs/Header` 를, `TaskStatus.msg` 가 `builtin_interfaces/Time` 을 쓰므로 `DEPENDENCIES` 선언과 `package.xml` 의 `<depend>` 가 필요하다. 레퍼런스 `od_msg` 는 기본 타입만 써서 이게 없었다.

같은 패키지 안의 msg 를 참조할 때는 패키지명 없이 타입명만 쓴다 — `DetectionArray.msg` 의 `DetectedObject[] objects`, `GetComponentPose.srv` 의 `DetectedObject source` 가 그 경우다.

---

## 2. 비전 ↔ 로봇 — 토픽 + 서비스 하이브리드

검출은 **토픽으로 상시 발행**하고, 좌표는 **서비스로 요청 시 확정**한다. 두 통신 방식을 섞는 이유가 이 절의 핵심이다.

| | 통신 | 이유 |
| --- | --- | --- |
| 검출 결과 | 토픽 `/detection/objects` | 요청과 무관하게 계속 나와야 관찰·디버깅이 된다. 로봇을 세워둔 채 비전만 볼 수 있다 |
| 파지 자세 | 서비스 `/get_component_pose` | 로봇이 움직이기 직전 **그 시점**에 확정되어야 한다. 비동기로 흘러오면 언제 것인지 모른다 |

### 2.1 왜 레퍼런스의 `SrvDepthPosition` 을 그대로 쓰지 않는가

레퍼런스 계약은 이렇다.

```
string target
---
float64[] depth_position
```

문제가 둘이다.

**첫째, 1회 호출 = 1개 품목이다.** 공구 하나 집어오는 작업엔 충분했지만 키팅에는 맞지 않는다. 레시피가 "컵라면 2 + 마스크 1" 이면 호출마다 다시 촬영·추론이 돌고, 그 사이 팔이 움직이며 장면이 바뀐다.

**둘째, 검출과 좌표 변환이 한 덩어리다.** 검출 결과를 따로 볼 방법이 없어서, 파지가 빗나갔을 때 YOLO 가 틀렸는지 캘리브레이션이 틀렸는지 가릴 수 없다.

토픽/서비스 분리가 둘 다 해결한다. 검출은 토픽에 항상 떠 있고(관찰 가능), 좌표 변환은 별도 노드의 서비스다(책임 분리).

### 2.2 `msg/DetectedObject.msg`

```
# 검출된 물체 하나. 좌표계는 카메라 기준, 단위 mm.
string     class_name      # class_names.json 의 이름. 예: "cup_ramen"
float32    score           # 0.0 ~ 1.0
float64[3] camera_xyz      # 카메라 좌표계 (x, y, z), mm
float64[]  masking_map     # polygon 마스킹맵
int32[2]   centroid_px     # seg mask 무게중심 픽셀 (x, y). 로깅/디버깅용
```

**`camera_xyz` 까지 `object_detection` 이 채운다.** 픽셀만 발행하고 좌표 변환을 나중에 하는 방식도 가능하지만, 그러면 color 프레임(mask)과 depth 프레임의 시각 동기를 `message_filters` 로 따로 맞춰야 한다. `object_detection` 노드 하나만이 color·depth·camera_info 를 **모두** 갖고 있으므로, 여기서 역투영까지 끝내면 세 프레임의 동기가 자동으로 맞는다. 동기화 코드를 아예 안 쓰는 게 가장 확실하다.

객체탐지를 진행해서 얻은 polygon 마스킹맵을 받아 가장 짧은 파지 거리를 측정한다.


### 2.3 `msg/DetectionArray.msg` → 토픽 `/detection/objects`

```
std_msgs/Header  header    # 검출에 사용한 프레임의 시각. 최신성 판정의 근거
DetectedObject[] objects   # 이번 프레임의 전체 검출. 없으면 빈 배열
```

**계약 세부:**
- `header.stamp` 는 **검출에 쓴 color 프레임의 stamp** 를 그대로 옮긴다. 발행 시각이 아니다. 추론에 걸린 시간만큼 차이가 나므로 이걸 틀리면 최신성 판정이 무의미해진다.
- depth 가 무효(0)인 검출은 **발행하지 않는다.** 좌표 없는 검출을 흘리면 로봇이 (0,0,0) 으로 움직이려 한다. 레퍼런스가 `sum(result) == 0` 으로 방어하던 걸 계약 수준에서 없앤다.
- 같은 클래스가 여러 개 보이면 모두 담는다. 몇 개를 집을지는 controller 가 레시피를 보고 정한다.
- 발행 주기 목표 2~5 Hz.

**QoS:** RELIABLE, `depth=1`. **최신 검출만 의미가 있다** — 큐에 쌓인 과거 검출은 유해하다. 카메라 원본 이미지 토픽은 BEST_EFFORT (`reference/subscriber_sourcecode/subscriber_img.py` 의 프로파일).

### 2.4 `srv/GetComponentPose.srv` → `position_estimation` 이 서버

```
string     component        # 클래스 이름
float64[6] robot_posx       # 촬영 시점 로봇 자세 (mm, deg). controller 가 채운다
float64    max_age_sec      # 이보다 오래된 검출은 거부. 0 이면 기본값
string[]   exclude_taken    # 이미 집어간 위치 제외용 (선택)
---
bool           success
float64[6]     target_pose      # 베이스 좌표 파지 자세 (mm, deg)
DetectedObject source           # 응답 근거가 된 원본 검출
string         error_code       # 아래 표 참조
```

**`source` 를 함께 돌려주는 이유:** 로봇이 엉뚱한 데로 갔을 때 "어떤 검출을 보고 그 좌표가 나왔는지" 가 응답 안에 남아 있어야 한다. `target_pose` 만 있으면 사후 분석이 불가능하다.

| `error_code` | 의미 | controller 대응 |
| --- | --- | --- |
| `""` | 성공 | 파지 진행 |
| `not_detected` | 해당 클래스가 검출에 없음 | 관찰 자세 재정착 후 재요청 |
| `stale` | 검출이 `max_age_sec` 보다 오래됨 | 정착 대기 후 재요청 |
| `out_of_workspace` | 변환 좌표가 작업영역 밖 | **움직이지 않는다.** 건너뛰고 기록 |
| `no_candidate` | 후보가 `exclude_taken` 으로 전부 소진 | 건너뛰고 `missing` 기록 |

### 2.5 현재 posx 를 request 로 넘기는 이유

`position_estimation` 이 hand-eye 변환을 하려면 **촬영 시점의 로봇 자세**가 필요하다. 카메라가 팔에 붙어 있어서(eye-in-hand) 팔이 움직이면 카메라도 움직이기 때문이다. 이 값을 얻는 경로가 셋 있다.

| 경로 | 판정 |
| --- | --- |
| position_estimation 이 `get_current_posx()` 호출 | **불가.** `DSR_ROBOT2` 를 import 하면 `DR_init.__dsr__node` 전역 싱글턴을 controller 와 다투게 된다 |
| TF2 로 `base ← camera` lookup | 정석이지만, 두산 드라이버가 해당 TF 를 발행하는지 확인이 필요하다 |
| **controller 가 request 에 담아 전달** | **채택.** controller 는 이미 posx 를 안다 |

**근거:** `reference/` 전체에서 로봇 자세를 얻는 코드는 예외 없이 `get_current_posx()` API 호출이다 (`robot_control.py`, `rokey_cobot2/basic/get_current_pos.py`, `jog_complete.py`). posx 를 담은 상태 토픽이나 TF 를 쓰는 예가 하나도 없다. 즉 **토픽/TF 가 있다는 근거가 없다.** 10일 일정에서 확인되지 않은 전제 위에 크리티컬 패스를 올리지 않는다.

request 로 넘기면 `position_estimation` 은 로봇 API 를 전혀 import 하지 않는 **순수 계산 노드**가 된다. DR_init 충돌이 원천적으로 없고, 로봇 없이 단위 검증이 가능하다.

> Day 1 에 `ros2 topic list | grep dsr` 로 상태 토픽 존재를 확인한다. 있으면 position_estimation 이 구독하고 `robot_posx` 는 오버라이드용으로 남는다. **어느 쪽이든 계약은 그대로**라 팀 재빌드가 발생하지 않는다.

### 2.6 최신성 가드 — 생략 불가

eye-in-hand 구성에서 가장 위험한 실패다. position_estimation 이 들고 있는 마지막 검출이 팔 이동 **전** 프레임이면, 그때의 카메라 좌표에 **지금**의 posx 를 곱하게 된다. 좌표가 통째로 틀리고, 로봇은 그걸 모른 채 내려간다.

세 겹으로 막는다.

1. `DetectionArray.header.stamp` — 검출 프레임의 시각을 계약에 넣는다
2. request 의 `max_age_sec` — position_estimation 이 나이를 검사하고 초과 시 `stale` 반환
3. controller 가 관찰 자세 도착 후 **정착 대기(settle)** 를 거친 뒤 요청 — 이동 직후의 프레임을 애초에 쓰지 않는다

`detection_age` 를 응답에 담아 controller 가 로그에 남긴다. 조용히 넘어가는 실패를 만들지 않는다.

### 2.7 `srv/InspectKit.srv` → `position_estimation` 이 서버

작업 완료 후 키팅 트레이를 재촬영해 실제 구성 결과를 판정한다. 기획서의 "작업 실행 여부가 아니라 실제 키트 구성 결과를 기준으로 성공 판정" 요구를 담는 인터페이스다.

```
string[] expected_classes   # 기대 품목
int32[]  expected_counts    # 기대 수량. expected_classes 와 길이 동일
float64  max_age_sec        # 최신성 기준
---
bool     ok                 # 기대와 실제가 완전히 일치
string[] missing            # 부족한 품목 (수량 부족 포함)
string[] unexpected         # 레시피에 없는데 들어간 품목 (오투입)
int32[]  actual_counts      # expected_classes 순서에 맞춘 실제 개수
float64  detection_age
```

서버를 `position_estimation` 에 두는 이유: 이미 `/detection/objects` 를 구독하고 있고 최신성 검사 로직도 거기 있다. 검사는 좌표가 필요 없는 단순 카운팅이지만, **최신성 판정은 똑같이 필요하다.** 같은 가드를 두 노드에 중복 구현하지 않는다.

`actual_counts` 를 따로 주는 이유: `ok=false` 일 때 "0개라 없는 건지 1개만 들어간 건지" 를 구분해야 재시도 판단이 된다.

---

## 3. 음성 ↔ 로봇

### 3.1 `srv/GetCommand.srv` → `command_node` 가 서버

```
---
bool   success
string command_json
string error_code
```

요청 필드가 없다. 레퍼런스가 `std_srvs/Trigger` 를 쓰던 자리인데, 응답 구조가 달라서 전용 srv 로 만든다.

> **웨이크워드 감지 범위 — 팀원과 확인 필요.** controller 는 `LISTEN` 상태에서만 이 서비스를 호출하고, 요청을 하나 보낸 뒤 응답이 올 때까지 다음 요청을 보내지 않는다. 즉 `VALIDATE`~`REPORT` 구간에는 이 서비스 자체를 호출하지 않으므로, **`command_node` 의 웨이크워드 감지가 이 서비스 콜백 안에서만 동작한다는 전제**하에 실행 중 발화가 자동으로 무시된다. 만약 `command_node` 가 요청 유무와 무관하게 백그라운드로 항상 마이크를 열어놓는 구조라면, EXECUTE 중 발화가 어딘가에 버퍼링됐다가 다음 `LISTEN` 에서 갑자기 튀어나올 수 있다 — 이 경우 계약을 다시 봐야 한다.

**`command_json` 스키마** (success=true 일 때만 유효):

```json
{
  "kit_type": "earthquake",
  "items": [
    {"name": "cup_ramen", "qty": 2},
    {"name": "mask",      "qty": 1}
  ],
  "raw_text": "지진 키트로 컵라면 두 개랑 마스크 하나 담아줘"
}
```

- `name` 은 반드시 `class_names.json` 에 존재하는 클래스여야 한다. **검증 책임은 음성 노드에 있다.** LLM 출력을 그대로 흘리지 않는다 (기획서의 "명령 검증기" 가 이 지점이다).
- `qty` 는 1 이상 정수.
- `raw_text` 는 DB 기록·디버깅용. 로봇은 쓰지 않는다.
- JSON 파싱 실패나 스키마 위반은 **로봇 쪽에서도 한 번 더 방어한다.** 신뢰 경계이므로 양쪽에서 검증한다.

**`error_code`** (success=false 일 때):

| 코드 | 의미 | 로봇 동작 |
| --- | --- | --- |
| `wakeword_timeout` | 웨이크워드 미감지 | 조용히 IDLE 복귀 |
| `stt_failed` | 음성 인식 실패 | IDLE 복귀, 재시도 안내 |
| `invalid_command` | 검증기가 거부 (미지원 품목/수량) | IDLE 복귀, 사유 로그 |
| `openai_quota_exhausted` | 크레딧 소진 | **에러 로그 + 중단.** 재시도해도 안 풀린다 |
| `openai_rate_limit` | 일시적 제한 | 잠시 후 재시도 가능 |
| `openai_error` | 기타 API 오류 | IDLE 복귀 |

이 코드 체계는 레퍼런스 `get_keyword.py` 가 이미 쓰던 것을 그대로 승계한다. 크레딧 소진과 레이트 리밋을 구분하는 게 실전에서 유효했다 — 전자는 기다려도 안 풀린다.

> **[열린 이슈 — DB 담당 팀원 확인 필요] `kit_type` → 레시피 자동 조회.**
> 지금은 LLM이 발화에서 `kit_type` 과 `items` 를 함께 추출하는데, `items` 처럼 `class_names.json` 대조 검증이 있는 것과 달리 `kit_type` 은 검증 파일이 없어 자유생성 문자열이 그대로 통과한다. "지진 키트 조립해줘"처럼 `kit_type` 만 말해도 미리 정의된 품목 구성(컵라면 2개, 마스크 1개 등)으로 `items` 를 채우는 방향이 논의됐다 — 다만 [05 DB 전체 구조](05-db-전체-구조.md)에 정의된 3개 컬렉션(`commands`/`kit_executions`/`component_executions`)에는 아직 이 kit_type→품목 매핑(레시피) 자체를 담을 테이블/컬렉션이 없다. 실제 스키마·조회 방식은 DB 담당 팀원이 확정한다.

---

## 4. 상태 발행 (로봇 → DB/UI)

### 4.1 `msg/TaskStatus.msg`

```
string task_id                 # 작업 1회의 UUID
string state                   # 상위 상태머신 (03 문서 2절)
string kit_type
string component               # 지금 처리 중인 Component 이름. 없으면 빈 문자열
int32  component_index         # 0-based
int32  component_total         # flatten 후 Component 개수 (수량이 펼쳐진 값)
int32  attempt                 # 이 Component 의 시도 횟수 (1-based)
string component_state         # PENDING | DONE | SKIPPED
string detail                  # 사람이 읽는 부가 설명 / 실패 사유(error_code)
builtin_interfaces/Time stamp
```

토픽: `/kit/task_status`, QoS 는 기본 `depth=10` reliable.

**`component_total` 은 flatten 후 개수다.** 레시피가 "컵라면 2 + 마스크 1" 이면 3 이지 2 가 아니다. 진행률을 Component 단위로 보여야 실제 작업량과 맞는다 ([03 문서 3.1절](03-system-flow.md)).

`attempt` 와 `component_state` 를 담는 이유: 재시도가 Component 안에서 일어나므로, 이 둘이 없으면 DB 에 "성공/실패" 만 남고 **몇 번 만에 됐는지**가 사라진다. 기획서의 정량 평가 항목(품목별 파지 성공률)을 채우려면 시도 횟수가 필요하다.

### 4.2 DB 를 직접 건드리지 않는다

기획서에 PostgreSQL 적재 요구가 있지만, **로봇 노드는 DB 를 모른다.** `/kit/task_status` 를 발행할 뿐이고 DB 담당 팀원이 이 토픽을 구독해 적재한다.

이유는 두 가지다. 첫째, DB 가 죽거나 느릴 때 로봇이 멈추면 안 된다. 물리적으로 움직이는 장비에 blocking I/O 를 물리지 않는다. 둘째, DB 스키마 변경이 로봇 코드 변경으로 번지지 않는다.

레퍼런스 `robot_control.py` 의 `/ui/current_task` (`std_msgs/String` 에 JSON 을 넣던 방식) 를 타입 있는 msg 로 승격한 것이다. String+JSON 은 필드 오타가 런타임까지 안 잡힌다.

---

## 5. 계약 검증 방법

노드 구현 전에도 계약이 맞는지 확인할 수 있다.

```bash
# 인터페이스 정의 확인
ros2 interface show kit_interfaces/srv/GetComponentPose
ros2 interface show kit_interfaces/msg/DetectionArray

# 검출 토픽 관찰 (mock 발행 중이어도 보인다)
ros2 topic echo /detection/objects
ros2 topic hz   /detection/objects

# 좌표 서비스 왕복 시험 (Day 2)
ros2 service call /get_component_pose kit_interfaces/srv/GetComponentPose \
  "{component: 'cup_ramen', robot_posx: [400,0,400,0,180,0], max_age_sec: 1.0}"
ros2 service call /get_command kit_interfaces/srv/GetCommand "{}"

# 상태 발행 확인
ros2 topic echo /kit/task_status

# Day 1 확인 항목: 로봇 상태 토픽이 실제로 있는지 (2.5절)
ros2 topic list | grep dsr
```

Day 2 에 `object_detection` 이 고정 `DetectionArray` 를 발행하는 mock 부터 만드는 이유가 이것이다. YOLO 모델이 나오기 전에 계약과 `position_estimation` 결선을 먼저 끝내둔다.
