# DB 저장 대상 및 프로젝트 로그 구조

관련 이슈·To-do: DB 저장 대상과 테이블 관리 범위 구체화 (https://app.notion.com/p/DB-3ce0f4c1b9cc80c7a51ce2ee252ecc6b?pvs=21), 인터페이스 정의 (https://app.notion.com/p/3cf0f4c1b9cc8018bcf8ce50859df357?pvs=21), RDBMS 사용 적합성 (https://app.notion.com/p/RDBMS-3cf0f4c1b9cc803f85aee21697319f29?pvs=21)
기록일: 2026년 9월 1일
담당자: 봉승현
마지막 수정: 2026년 9월 3일 오후 5:26
분류: 설계 결정
분야: Backend, DB
생성일: 2026년 9월 1일 오후 9:42
요약: 작업 레시피, 실행 결과, 모델 추론과 오류 이력 중 DB에 저장할 범위 및 테이블 구조 결정
작성 상태: 정리 완료

## 결정할 문제

- DB 구조 및 저장 데이터 종류
    - RDBMS — PostgreSQL
    - NoSQL — MongoDB
    - 각 데이터별 목적에 따라 독립적으로 두 DB를 운용한다.
- 프로젝트 로그 구조
    - 어디서 로그를 띄우고, 언제 받아올 것인지에 대한 연결 구조 정립 필요

## 저장 대상 후보

| 데이터 | 저장 목적 | 생성 주기 | 보존 여부 | 비고 |
| --- | --- | --- | --- | --- |
| 작업 레시피 | 작업 기준 정보 |   • 최초 생성
  • 작업 중 음성으로 생성 | 보존 | Edge Goal
초기 단계 개발 X |
| 수량 및 재고관리 | 작업 기준 정보 |   • 최초 생성
  • 작업 중 변경 사항 발생 | 보존 |  |
| 명령 해석 결과 | 문장 인식률, 키워드 인식률 평가 | 음성 명령이 전달되고 파싱이 진행될 때마다 생성 | 보존 | command node에서 생성 |
| YOLO 추론 결과 | 클래스 별 객체 탐지 성능 평가 | 실시간 생성 | 선별 보존(position estimation에서 활용한 시점만) | object detection node에서 생성
topic 형태로 발행됨 |
| 각 component 별 작업 과정 기록 | 클래스 별 파지 성공률 평가
성능 디버깅 | execution 생성 후 controller의 EXECUTE 상태에서 생성 | 보존 | controller node에서 생성 |
| 전체 kit 단위  작업 기록 | 시스템 성공률, 안정성 평가 | 전체 execution이 완료되고, controller의 REPORT 상태에서 생성 | 보존 | controller node에서 생성 |

## 대안 비교

MongoDB

PostgreSQL

## 최종 결정

PostgreSQL은 관계와 정합성이 중요한 품목·재고 데이터에 사용한다.

MongoDB는 구조가 서로 다른 Command, Component Execution, Kit Execution 데이터를 문서 단위로 저장하여 실행 추적과 사후 데이터 분석에 활용한다.

| DB | 저장 대상 | 역할 |
| --- | --- | --- |
| PostgreSQL | Item, Inventory | 품목 기준정보 및 현재 재고 관리 |
| MongoDB | Command | 문장·키워드 인식 결과 저장 |
| MongoDB | Component Execution | 객체 검출·파지·배치 과정 추적 |
| MongoDB | Kit Execution | 전체 키트 실행 상태와 최종 결과 저장 |
| Python 분석 파일 | 시나리오 정답과 DB 데이터 비교 | 인식률·성공률·안정성 분석 |
- 저장 대상: 품목 기준정보 및 현재 재고 관리, 문장/키워드 인식 결과, 객체 검출/파지/배치 과정 추적, 전체 키트 실행 상태와 최종 결과 저장
- 제외 대상: 인식률, 성공률, 안정성 등 계산이 필요한 항목, 작업 레시피
- 테이블 및 컬렉션:
    - PostgreSQL
        - item
        - inventory
    - MongoDB
        - commands
        - kit_executions
        - component_executions
- 작업·실행 추적 ID: task_id
    - 전체 데이터는 task_id로 연결된다.
    - component는 task_id + component_index로 식별한다.
    - 재시도는 components_executions 문서 내부의 attempts 배열에 저장한다.

### 전체 결정 구조

**MongoDB와 연결되는 항목들**

| 토픽 | 발행 노드 | 발행 시점 | DB 저장 대상 |
| --- | --- | --- | --- |
| `/kit/command_result`  | `command_node`  | STT·LLM·검증 완료 시 | `commands` |
| `/kit/task_status` | `controller` | Kit 상태 또는 진행 대상 변경 시 | `kit_executions` |
| `/kit/component_result` | `controller` | Component 최종 종료 시 | `component_executions` |

![image.png](DB%20%EC%A0%80%EC%9E%A5%20%EB%8C%80%EC%83%81%20%EB%B0%8F%20%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8%20%EB%A1%9C%EA%B7%B8%20%EA%B5%AC%EC%A1%B0/image.png)

#### `/kit/command_result`

음성 입력부터 명령 검증까지의 결과를 저장하는 토픽

명령 처리 한 건이 끝났을 때 한 번 발행(검증 단계까지 완료 후)

```
# CommandResult.msg

string task_id

bool success
string raw_text
string command_json

string validation_result
string error_code
string detail

builtin_interfaces/Time stamp
```

#### `/kit/task_status`

전체 Kit 작업이 지금 어느 단계까지 진행이 되었고, 최종 결과가 무엇인지 추적하는 토픽

Controller 상태 변경, 현재 Component 변경, kit 종료 시 발행

```
# TaskStatus.msg

string task_id

string state
string task_status
string kit_type

string current_component
int32 current_component_index
int32 component_total

string inspection_result
string error_code
string detail

builtin_interfaces/Time stamp
```

각 단계에 따라 다음 메시지가 전송되고, DB에 순차적으로 처리된다.

- kit_executions 문서 생성, status = RUNNING
- current_state 갱신, 현재 Component 갱신, status_history 추가
- status = SUCCESS 또는 FAILED, inspection_result 저장, ended_at 저장

#### `/kit/component_result`

Component 하나의 최종 작업 결과와 그 과정에서 발생한 재시도를 저장하는 토픽

component 하나가 최종 종료될 때 한 번 발행(SUCCESS, FAILED, SKIPPED 중 하나)

```
# ComponentResult.msg

string task_id

int32 component_index
int32 component_total
string component
string slot

string status
int32 attempt_count
string attempts_json

string error_code
string detail

builtin_interfaces/Time started_at
builtin_interfaces/Time ended_at
```

```json
# attempt_json 예시 구조
[
  {
    "attempt_no": 1,
    "result": "FAILED",
    "detection": {
      "score": 0.82,
      "camera_xyz": [31.2, -22.4, 518.0],
      "centroid_px": [314, 228],
      "detection_age": 0.21
    },
    "target_pose": [
      421.1,
      -125.2,
      91.3,
      0.0,
      180.0,
      0.0
    ],
    "grasp": {
      "success": false,
      "gripper_width": 0.0
    },
    "release": null,
    "error_code": "empty_grasp",
    "started_at": "2026-09-03T01:20:31.421Z",
    "ended_at": "2026-09-03T01:20:35.182Z"
  },
  {
    "attempt_no": 2,
    "result": "SUCCESS",
    "detection": {
      "score": 0.91,
      "camera_xyz": [33.8, -20.1, 515.0],
      "centroid_px": [318, 226],
      "detection_age": 0.18
    },
    "target_pose": [
      423.5,
      -122.9,
      89.1,
      0.0,
      180.0,
      0.0
    ],
    "grasp": {
      "success": true,
      "gripper_width": 37.4
    },
    "release": {
      "pose": [
        520.0,
        -160.0,
        120.0,
        0.0,
        180.0,
        0.0
      ],
      "slot": "slot_1",
      "success": true
    },
    "error_code": null,
    "started_at": "2026-09-03T01:20:36.104Z",
    "ended_at": "2026-09-03T01:20:42.528Z"
  }
]
```

실패로 인해 얻지 못한 데이터는 null로 비워둔 채 전송한다.

**PostgreSQL에 연결되는 항목들**

| 조건 | 재고 변경 |
| --- | --- |
| Component `SUCCESS`이고 배치 성공 | 1개 차감 |
| Component `FAILED` | 변경 없음 |
| Component `SKIPPED` | 변경 없음 |
| Kit 최종 `FAILED` | 별도 변경 없음 |

#### `item`

Voice, Vision, Controller, DB가 공통으로 사용하는 품목 코드를 관리

| 항목 | 타입 | 비고 |
| --- | --- | --- |
| item_id | bigint | PK |
| item_code | varchar | NOT NULL, UNIQUE
vision의 class_name
componentResult의 component
와 동일한 값이어야 한다. |
| item_name | varchar | NOT NULL |

#### `inventory`

| 항목 | 타입 | 비고 |
| --- | --- | --- |
| item_id | bigint | PK/FK |
| quantity | integer | 기본값 0 |
| updated_at | timestamptz | 기본값 현재 시각 |

수량 차감은 DB 노드에서 관리한다.

## 영향 범위

- Backend: db 노드 생성하여 bridge 역할 동시 수행
- ROS2: controller에서 발행할 데이터 결정
- Dashboard: 구현 여부 미정

## 후속 작업

db 구조 설계 및 구축

| 데이터 | 저장 위치 |
| --- | --- |
| 품목 코드·이름·사용 여부 | PostgreSQL `item` |
| 품목별 현재 수량 | PostgreSQL `inventory` |

![work_flow_updated_db.png](DB%20%EC%A0%80%EC%9E%A5%20%EB%8C%80%EC%83%81%20%EB%B0%8F%20%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8%20%EB%A1%9C%EA%B7%B8%20%EA%B5%AC%EC%A1%B0/work_flow_updated_db.png)