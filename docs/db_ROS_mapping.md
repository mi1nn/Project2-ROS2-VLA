# DB 저장 계약

## 메시지, MongoDB 문서 매핑표

### CommandResult

| ROS 필드 | MongoDB 필드 | 변환 |
| --- | --- | --- |
| task_id | task_id | 그대로 |
| raw_text | raw_text | 그대로 |
| command_json | command | JSON 문자열을 파싱하여 중첩 문서로 저장 |
| success | validation.success | 그대로 |
| validation_result | validation.result | 그대로 |
| error_code | validation.error_code | 빈 문자열은 null |
| detail | validation.detail | 빈 문자열은 null |
| stamp | created_at | BSON datetime |

#### 저장 정책 :

- task_id 기준 최초 한 번만 저장
- 중복 메시지는 기존 문서를 덮어쓰지 않음
- `$setOnInsert` 사용

#### 검증 정책 :
- 빈 문자열 -> `command: null`
- 유효한 JSON 객체 -> 파싱하여 저장
- 잘못된 JSON 객체 -> 메시지 저장 거부 및 오류 로그

### TaskStatus

| ROS 필드 | MongoDB 필드 | 변환 |
| --- | --- | --- |
| task_id | task_id | 그대로 |
| state | current_state | task_id 기준 문서의 current_state 갱신 |
| state | status_history | 중첩 문서, state와 stamp 저장 |
| kit_type | kit_type | 그대로 |
| task_status | status | task_id 기준 문서의 status 갱신 |
| current_component | current_component.name | 그대로 |
| current_component_index | current_component.index | 그대로 |
| component_total | component_total | 그대로 |
| inspection_result | final_inspection | 빈 문자열이 아닐 때 JSON 파싱 후 중첩 문서로 저장 |
| error_code | error.code | 빈 문자열은 null |
| detail | error.detail | 빈 문자열은 null |
| stamp | 시간 관련 | datetime |

| 메시지 상황 | `stamp` 저장 위치 |
| --- | --- |
| 최초 `RUNNING` | `started_at` |
| 모든 상태 메시지 | `status_history[].timestamp` |
| 최종 `SUCCESS` 또는 `FAILED` | `ended_at` |
| 검사 JSON의 `inspected_at` | BSON datetime으로 변환 후 `final_inspection.inspected_at` |

#### 저장 정책 :

- 최초 메시지 : `$setOnInsert`로 문서 생성
- 이후 메시지 : `$set`으로 기존 문서 갱신

- inspection_result는 json 형식으로 다음과 같은 항목을 포함해서 보낸다.
    - 검사 실행 전에는 `""` 빈 문자열 전송
    - 유효한 json일 때 final_inspection을 갱신한다.
- 결과값 범위 : PASS, FAIL, ERROR

```json
{
  "result": "PASS",
  "expected_counts": {
    "cup_ramen": 2,
    "mask": 1
  },
  "actual_counts": {
    "cup_ramen": 2,
    "mask": 1
  },
  "missing": [],
  "unexpected": [],
  "detection_age": 0.15,
  "inspected_at": "2026-09-04T02:30:12.421Z"
}
```

- 해당 값을 받아올 때 다음 검증을 거친다.
    - result는 PASS, FAIL, ERROR 중 하나
    - expected_counts는 객체
    - actual_counts는 객체 또는 null
    - missing과 unexpected는 배열
    - detection_age는 0 이상 숫자 또는 null
    - inspected_at은 BSON datetime으로 변환

### ComponentResult

| ROS 필드 | MongoDB 필드 | 변환 |
| --- | --- | --- |
| task_id | task_id | 그대로 |
| component_index | component_index | 그대로 |
| component_total | component_total | 그대로 |
| component | class_name | 그대로 |
| slot | slot | 그대로 |
| status | status | 그대로 |
| attempt_count | attempt_count | 그대로 |
| attempts_json | attempts | JSON 파싱 후 배열 저장 |
| error_code | error.code | 빈 문자열은 `null` |
| detail | error.detail | 빈 문자열은 `null` |
| started_at | started_at | BSON datetime |
| ended_at | ended_at | BSON datetime |
- 해당 값을 받아올 때 다음을 검증한다.
    - attempt_count == attempts 배열 길이
    - attempt_no는 1부터 순서대로 증가
    - status가 SUCCESS이면 마지막 Attempt도 SUCCESS
    - attempts_json은 반드시 JSON 배열
    - SKIPPED이고 시도하지 않았다면 []

#### 저장 정책 :

- Component는 task_id + component_index로 식별
- Component 최종 종료 시 한 번만 저장
- 중복 메시지는 기존 문서를 덮어쓰지 않음
- $setOnInsert 사용

### 재고 차감

#### 재고 차감 정책 :
- ComponentResult.status == SUCCESS
- 최초로 저장된 ComponentResult
- 두 조건을 충족하면 해당 component 품목 재고 1개 차감

