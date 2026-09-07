# 02. 인터페이스 계약 (`kit_interfaces`)

관련 문서: [01 아키텍처](01-architecture.md) · [03 시스템 플로우](03-system-flow.md) · [05 데이터베이스](05-database.md)

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
    CommandResult.msg
    ComponentResult.msg
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
  "msg/CommandResult.msg"
  "msg/ComponentResult.msg"
  "msg/TaskStatus.msg"
  "srv/GetComponentPose.srv"
  "srv/InspectKit.srv"
  "srv/GetCommand.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)
```

`DetectionArray.msg`가 `std_msgs/Header`를, 세 DB 이벤트 메시지가 `builtin_interfaces/Time`을 쓰므로 `DEPENDENCIES` 선언과 `package.xml` 의 `<depend>` 가 필요하다. 레퍼런스 `od_msg` 는 기본 타입만 써서 이게 없었다.

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

**QoS:** BEST_EFFORT, `depth=1`. **최신 검출만 의미가 있다** — 큐에 쌓인 과거 검출은 유해하다. 카메라 원본 이미지 토픽과 동일 정책 (`reference/subscriber_sourcecode/subscriber_img.py` 의 프로파일).

> **2026-09-04 수정: RELIABLE → BEST_EFFORT.** 애초에 "가공된 신호라 유실 비용이 크다"는 감으로 RELIABLE을 기본값으로 잡았는데, 다시 보니 근거가 약했다. `depth=1`이라 옛 프레임은 어차피 버려지고, `position_estimation`이 `max_age_sec`로 신선도를 앱 레벨에서 이미 검사한다 — 한 틱 유실돼도 0.3s 뒤 다음 발행이 덮어쓰므로 트랜스포트의 재전송 보장이 실제로 하는 일이 없다. 오히려 RELIABLE의 ACK/재전송이 최신 프레임 전달을 지연시킬 여지만 있어, 원본 카메라 토픽과 정책을 통일했다.

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
| `out_of_workspace` | 변환 좌표가 작업영역 밖 | **움직이지 않는다.** `FAILED`로 기록하고 다음 Component 진행 |
| `no_candidate` | 후보가 `exclude_taken`으로 전부 소진 | Component FAILED 후 다음 품목. 실제 부족 여부는 최종 검사로 확인 |

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

Controller는 error_code=stale로 좌표 검출 노후를 처리한다. 검사 서비스의 detection_age만 검사 JSON에 기록한다.

**Controller 초기 정책:** max_age_sec=1.0, 관찰·검사 정착 대기 1.2초를 사용한다.
정착 시작은 자세 이동 완료 시점이며 응답을 받을 때까지 해당 자세를 유지한다.
검출 stamp와 서버 시각이 동일 시간 기준이고 이동 완료 시점을 정확히 안다는 전제다.
호스트 간 시계 오차도 0.2초 여유 이내여야 한다. 미래 stamp·시계 불일치 방어가 완료되었다고
가정하지 않는다. 정착 시간이 허용 검출 나이보다 길도록 함께 조정하고 실기에서 검증한다.

현재 서버는 촬영 시각 하한 요청이나 새 프레임 대기 기능 없이 캐시를 즉시 판정한다.
초기 Controller는 exclude_taken=[]를 보낸다. 기존 제외 키는 픽셀 중심 문자열이므로
여러 프레임에 걸쳐 안정적인 물체 ID로 누적하지 않는다. 서버의 기존 제외 기능은 유지한다.

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

**현재 구현과 Controller 판정:** InspectKit에는 success/error_code가 없다.
검출 캐시가 없으면 ok=false, detection_age=inf이며 오래된 검출도 ok=false로 반환한다.
나이가 유한하고 0 이상이며 요청한 max_age_sec 이내인지 먼저 확인한다. 범위를 벗어나거나 통신 timeout·future 예외·응답 형식 오류이면 ERROR다.
유효한 응답만 ok=true → PASS, ok=false → FAIL로 해석한다.

actual_counts 길이는 expected_classes와 같고 수량은 음수가 아니어야 한다.

expected_classes와 expected_counts는 원래 검증된 명령에서 동일 순서로 만든다.
실패한 Component를 기대 수량에서 빼지 않는다. Task는 검사 PASS이고 TASK_FATAL·최종 복귀 실패가 없을 때 SUCCESS다.

현재 inspect_counts는 전체 검출을 세며 트레이 ROI 필터는 없다. 초기 운영에서는 검사 화면에 완성 트레이의 검사 대상 물체만 포함되도록 배치한다. 원본 물체가 함께 보이면 검사 성공 판정에 사용하기 전에 촬영 구성을 조정한다. ROI 필터 추가는 이번 범위 밖이다.

---

## 3. 음성 ↔ 로봇

### 3.1 `srv/GetCommand.srv` → `command_node` 가 서버

```
string task_id   # 작업 1회 식별자. controller 가 생성해 요청에 담는다.
---
bool   success
string command_json
string error_code
```

레퍼런스가 `std_srvs/Trigger` 를 쓰던 자리인데, 응답 구조가 달라서 전용 srv 로 만든다.

**`task_id`.** MongoDB `commands`/`kit_executions`/`component_executions` 세 컬렉션을 하나의
작업으로 묶는 키다([05 데이터베이스](05-database.md) ID 체계). `controller` 가
`IDLE → LISTEN` 진입 시(이 서비스를 호출하는 유일한 지점) 생성해서 요청에 실어 보낸다.
`command_node` 는 응답과 `/kit/command_result`에 이 값을 그대로 사용하므로, 이후 DB 기록
단계에서 재발급하지 않고 요청 시점의 값을 계속 쓴다. `command_json` 내부에는 실행 명령인
`kit_type`과 `items`만 넣고, `task_id`와 `raw_text`는 `CommandResult`의 별도 필드로 전달한다.

**웨이크워드 감지 범위 — 현재 구현 확인.** get_command 콜백 내부에서 마이크를 열어 최대 30초 동안 웨이크워드를 기다린 뒤 닫는다. Controller는 LISTEN에서 요청을 하나만 보낸다. 클라이언트 timeout은 서버 콜백 취소를 뜻하지 않으므로 아래 재시작 정책을 따른다.

**반복 작업:** 응답 반환 후 음성 노드는 다음 서비스 요청을 기다린다. 스스로 웨이크워드 대기를 다시 시작하지 않는다. Controller의 VALIDATE~REPORT 동안에는 새 명령 요청이 없고, 키팅이 끝나 IDLE → LISTEN으로 돌아가야 마이크 감지가 다시 시작된다. wakeword_timeout은 실패 응답이며 노드 종료가 아니다. Controller의 명령 응답 제한 60초도 키팅 시간을 포함하지 않는다.
다만 현재 is_wakeup/close 예외는 실패 응답으로 감싸지 않으므로 별도 보완 대상이다.

**`command_json` 스키마** (success=true 일 때만 유효):

```json
{
  "kit_type": "earthquake",
  "items": [
    {"name": "cup_ramen", "qty": 2},
    {"name": "mask",      "qty": 1}
  ]
}
```

- `name` 은 반드시 `class_names.json` 에 존재하는 클래스여야 한다. **검증 책임은 음성 노드에 있다.** LLM 출력을 그대로 흘리지 않는다 (기획서의 "명령 검증기" 가 이 지점이다).
- `qty` 는 1 이상 정수.
- `raw_text`는 `command_json`이 아니라 `CommandResult.raw_text`에 담아 DB 기록·디버깅에 사용한다.
- JSON 파싱 실패나 스키마 위반은 **로봇 쪽에서도 한 번 더 방어한다.** 신뢰 경계이므로 양쪽에서 검증한다.

**현재 구현과 예정 변경:** 현재 음성 노드는 command_json에 raw_text와 task_id도 넣는다.
이는 위 목표 계약과 다른 과도기 구현이며 해당 필드를 빼는 수정은 음성 노드에서 추후 진행한다.
그 전까지 Controller는 kit_type/items를 검증하고 추가 메타데이터는 실행 해석에서 무시한다.
응답 task_id로 Controller가 생성한 task_id를 덮어쓰지 않는다. 이번 수정에서는 음성 코드를 변경하지 않는다.

**`error_code`** (success=false 일 때):

| 코드 | 의미 | 로봇 동작 |
| --- | --- | --- |
| `wakeword_timeout` | 웨이크워드 미감지 응답 | REPORT에서 FAILED 기록 후 IDLE 재대기 |
| `stt_failed` | 음성 인식 실패 응답 | REPORT에서 FAILED·사유 기록 후 IDLE 재대기 |
| `invalid_command` | 명령 거부 응답 | REPORT에서 FAILED·사유 기록 후 IDLE 재대기 |
| `openai_quota_exhausted` | 크레딧 소진 응답 | REPORT에서 FAILED 기록, 자동 재시작 차단 |
| `openai_rate_limit` | 일시적 제한 응답 | REPORT에서 FAILED 기록, 재요청 간격 후 IDLE 재대기 |
| `openai_error` | 기타 API 오류 응답 | REPORT에서 FAILED 기록 후 IDLE 재대기 |

명령 요청의 클라이언트 timeout·future 예외·서비스 준비 timeout도 REPORT에서 FAILED로 종료하고 자동 재시작을 차단한다. 이전 음성 처리 종료 여부를 확인할 수 없거나 서비스가 정상 동작하지 않는 상태에서 새 요청을 반복하지 않는다. 운영자가 서버 상태와 원인을 확인한 뒤 재가동한다. 별도 재개 서비스나 긴급 중단 API는 이번 단계에서 추가하지 않는다.
응답으로 확인된 재대기 가능 실패는 서버 콜백이 종료된 경우다. 물리 동작을 시작하지 않았다면 REPORT에서 불필요한 복귀 동작 없이 결과를 확정할 수 있다. 클라이언트의 명령 구조 검증 실패도 FAILED 기록 후 재대기한다. 재대기 간격은 Controller 파라미터로 두며 openai_rate_limit도 즉시 반복 호출하지 않는다.

이 코드 체계는 레퍼런스 `get_keyword.py` 가 이미 쓰던 것을 그대로 승계한다. 크레딧 소진과 레이트 리밋을 구분하는 게 실전에서 유효했다 — 전자는 기다려도 안 풀린다.

> **[열린 이슈] `kit_type` → 레시피 자동 조회.**
> 지금은 LLM이 발화에서 `kit_type`과 `items`를 함께 추출한다. [05 데이터베이스](05-database.md)는 작업 레시피를 저장 범위에서 제외하므로, `kit_type`만으로 품목을 채우는 기능이 필요해지면 DB 스키마와 분리된 설정 파일 또는 별도 서비스 계약을 추가로 정해야 한다.

---

## 4. DB 저장 이벤트

DB 노드는 아래 세 토픽을 구독한다. MongoDB 필드 매핑, 검증 및 재고 차감 정책은
[05 데이터베이스](05-database.md)를 따른다.

### 4.1 `msg/CommandResult.msg` (음성 → DB)

아래는 목표 발행 계약이다. 현재 get_keyword.py의 서비스 응답은 구현되어 있으나 CommandResult publisher 연결은 후속 작업이다. Controller의 결과 토픽 두 개는 발행 구현이 되어 있다.

```
string task_id
bool success
string raw_text
string command_json
string validation_result
string error_code
string detail
builtin_interfaces/Time stamp
```

토픽은 `/kit/command_result`다. 명령 해석과 검증이 끝날 때 성공·실패 모두 발행한다.
성공 시 `command_json`은 비어 있지 않은 JSON 객체여야 한다.

### 4.2 `msg/TaskStatus.msg` (로봇 → DB/UI)

```
string task_id
string state                   # 상위 상태머신 (03 문서 2절)
string task_status             # RUNNING | SUCCESS | FAILED
string kit_type
string current_component       # 지금 처리 중인 Component 이름. 없으면 빈 문자열
int32  current_component_index # 0-based
int32  component_total         # flatten 후 Component 개수
string inspection_result       # 최종 검사 JSON. 검사 전에는 빈 문자열
string error_code
string detail
builtin_interfaces/Time stamp
```

토픽은 `/kit/task_status`, QoS는 기본 `depth=10` reliable이다. `component_total`은
flatten 후 개수다. `inspection_result`는 `result`, `expected_counts`, `actual_counts`,
`missing`, `unexpected`, `detection_age`, `inspected_at`을 포함하는 JSON이다.

result는 PASS/FAIL/ERROR, expected_counts는 품목별 객체, actual_counts는 품목별 객체 또는 null, missing/unexpected는 배열이다. inspected_at은 timezone을 포함한 ISO 시각이다.
검사 오류의 무효 수량은 actual_counts=null로 기록하고 inf/NaN/음수 detection_age는 null로 정규화한다. 검사 미수행 시 inspection_result는 빈 문자열이다. 현재 Component가 없으면
이름은 빈 문자열, index는 -1이다. IDLE을 제외한 상태 전이마다 RUNNING을, REPORT에서 복귀 결과를 반영한 최종 SUCCESS/FAILED를 한 번 발행한다. 이는 Controller 발행 횟수 규칙이며 전송·DB 저장의 정확히 한 번 보장을 뜻하지 않는다.

### 4.3 `msg/ComponentResult.msg` (로봇 → DB)

```
string task_id
int32 component_index
int32 component_total
string component
string slot
string status                  # SUCCESS | FAILED | SKIPPED
int32 attempt_count
string attempts_json           # Attempt 배열 JSON
string error_code
string detail
builtin_interfaces/Time started_at
builtin_interfaces/Time ended_at
```

토픽은 `/kit/component_result`다. Component가 최종 종료될 때 한 번 발행한다.
재시도 상세는 `attempts_json` 배열로 보존하며, 최초 저장된 `SUCCESS`만 재고 차감 대상이다.

기존 DB 검증대로 attempt_count는 배열 길이와 같고 attempt_no는 1부터 연속 증가한다.
SUCCESS의 마지막 Attempt는 SUCCESS여야 하며 SKIPPED의 Attempt 배열은 비어 있어야 한다.
TASK_FATAL로 미시작한 Component는 SKIPPED로 발행하며 시작·종료 시각은 건너뛰기로 확정한
시각을 사용한다. index는 0부터 시작한다. Task 검사 PASS만으로 FAILED Component 상태나
기존 재고 차감 결과를 소급 수정하지 않는다.

### 4.4 로봇 노드는 DB를 직접 건드리지 않는다

로봇 노드는 `/kit/task_status`와 `/kit/component_result`를 발행할 뿐이고 `kit_db`의
`db_node`가 이를 구독해 적재한다. 음성 노드도 같은 원칙으로 `/kit/command_result`만
발행한다. DB가 죽거나 느려도 로봇의 물리 동작을 blocking하지 않고, DB 스키마 변경이
로봇 코드로 번지는 것을 막는다.

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
ros2 service call /get_command kit_interfaces/srv/GetCommand "{task_id: 'TASK-20260905T053012123456Z'}"

# 상태 발행 확인
ros2 topic echo /kit/task_status
ros2 topic echo /kit/command_result
ros2 topic echo /kit/component_result

# Day 1 확인 항목: 로봇 상태 토픽이 실제로 있는지 (2.5절)
ros2 topic list | grep dsr
```

Day 2 에 `object_detection` 이 고정 `DetectionArray` 를 발행하는 mock 부터 만드는 이유가 이것이다. YOLO 모델이 나오기 전에 계약과 `position_estimation` 결선을 먼저 끝내둔다.
