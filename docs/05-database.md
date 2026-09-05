# 데이터베이스 설계 및 DB 노드

## 1. 목적과 범위

이 문서는 DB 선택, 스키마, ROS 메시지 매핑, 저장 정책, `kit_db` 실행과 검증
방법을 정의한다.

| 저장소 | 저장 대상 | 목적 |
| --- | --- | --- |
| PostgreSQL | `item`, `inventory` | 품목 기준정보와 현재 재고 관리 |
| MongoDB | `commands` | 명령 해석 및 검증 결과 보존 |
| MongoDB | `kit_executions` | Kit 상태와 최종 검사 추적 |
| MongoDB | `component_executions` | Component 실행과 재시도 추적 |

집계 지표는 분석 코드에서 계산한다. 작업 레시피와 원본 실시간 추론 결과는
현재 저장 범위에서 제외한다.

## 2. 데이터 흐름

| 토픽 | 발행 시점 | 저장 대상 |
| --- | --- | --- |
| `/kit/command_result` | 명령 해석·검증 완료 | `commands` |
| `/kit/task_status` | Kit 상태 또는 현재 Component 변경 | `kit_executions` |
| `/kit/component_result` | Component 최종 종료 | `component_executions`, `inventory` |

```text
ROS 메시지 -> DBNode -> mapper 검증 -> MongoRepository
                                   -> 신규 SUCCESS이면 InventoryRepository
```

검증 오류와 DB 작업 오류는 callback에서 기록하며 노드는 계속 실행한다.

## 3. 식별자와 상태

| 대상 | 식별 방식 |
| --- | --- |
| 명령, Kit 실행 | `task_id` |
| Component 실행 | `task_id + component_index` |
| 재시도 | `task_id + component_index + attempt_no` |

별도 Component 실행 ID는 만들지 않는다. 재시도는 Component 문서의
`attempts` 배열에 저장한다. Kit 상태는 `RUNNING`, `SUCCESS`, `FAILED`,
Component 최종 상태는 `SUCCESS`, `FAILED`, `SKIPPED`, 검사 결과는 `PASS`,
`FAIL`, `ERROR`를 사용한다.

## 4. PostgreSQL

### `item`

| 필드 | 타입 | 제약 |
| --- | --- | --- |
| `item_id` | `bigserial` | PK |
| `item_code` | `varchar(100)` | NOT NULL, UNIQUE |
| `item_name` | `varchar(200)` | NOT NULL |

`item_code`는 Vision `class_name` 및 `ComponentResult.component`와 동일하다.

### `inventory`

| 필드 | 타입 | 제약 |
| --- | --- | --- |
| `item_id` | `bigint` | PK, `item` FK, ON DELETE CASCADE |
| `quantity` | `integer` | NOT NULL, 기본값 0, 0 이상 |
| `updated_at` | `timestamptz` | NOT NULL, 기본값 현재 시각 |

차감은 `item_code`가 일치하고 `quantity > 0`인 행만 단일 `UPDATE`로
갱신한다. 품목이 없거나 재고가 0이면 갱신되지 않으며 음수가 되지 않는다.

| ID | 품목 | 초기 수량 |
| ---: | --- | ---: |
| 0 | 마스크 | 2 |
| 1 | 분유 | 20 |
| 2 | 샴푸리필 | 1 |
| 3 | 수세미 | 1 |
| 4 | 양갱 | 1 |
| 5 | 여행용티슈 | 1 |
| 6 | 일회용숟가락 | 4 |
| 7 | 컵라면 | 1 |
| 8 | 햄 | 1 |

원본 스키마와 seed는 `infra/postgres/init`에 있다.

## 5. MongoDB

초기화는 문서 데이터나 JSON Schema validator를 만들지 않고 고유 인덱스만
생성한다.

| 컬렉션 | 고유 인덱스 |
| --- | --- |
| `commands` | `{task_id: 1}` |
| `kit_executions` | `{task_id: 1}` |
| `component_executions` | `{task_id: 1, component_index: 1}` |

### `commands`

```json
{
  "task_id": "TASK-001",
  "raw_text": "컵라면 하나 담아줘",
  "command": {"kit_type": "earthquake", "items": [{"name": "컵라면", "qty": 1}]},
  "validation": {"success": true, "result": "VALID", "error_code": null, "detail": null},
  "created_at": "BSON datetime"
}
```

`task_id` 기준 `$setOnInsert`로 최초 문서만 저장한다.

### `kit_executions`

```json
{
  "task_id": "TASK-001",
  "current_state": "EXECUTE",
  "kit_type": "earthquake",
  "status": "RUNNING",
  "current_component": {"name": "컵라면", "index": 0},
  "component_total": 1,
  "error": {"code": null, "detail": null},
  "status_history": [{"state": "EXECUTE", "timestamp": "BSON datetime"}],
  "started_at": "BSON datetime"
}
```

최초 값은 `$setOnInsert`, 현재 상태는 `$set`, 이력은 `$addToSet`으로
처리한다. 최초 `RUNNING`은 `started_at`, 최종 상태는 `ended_at`을 기록한다.
`inspection_result`가 있으면 검증 후 `final_inspection`에 저장한다.

### `component_executions`

```json
{
  "task_id": "TASK-001",
  "component_index": 0,
  "component_total": 1,
  "class_name": "컵라면",
  "slot": "slot_1",
  "status": "SUCCESS",
  "attempt_count": 1,
  "attempts": [{"attempt_no": 1, "status": "SUCCESS"}],
  "error": {"code": null, "detail": null},
  "started_at": "BSON datetime",
  "ended_at": "BSON datetime"
}
```

`task_id + component_index` 기준 `$setOnInsert`로 한 번만 저장한다.

## 6. ROS 매핑 및 검증

ROS Time은 UTC BSON datetime으로 변환하고 빈 `error_code`, `detail`은
`null`로 변환한다.

### `CommandResult`

| ROS 필드 | MongoDB 필드 | 규칙 |
| --- | --- | --- |
| `task_id` | `task_id` | 비어 있지 않은 문자열 |
| `raw_text` | `raw_text` | 그대로 |
| `command_json` | `command` | JSON 객체로 파싱 |
| `success`, `validation_result` | `validation.success`, `validation.result` | 그대로 |
| `error_code`, `detail` | `validation.error_code`, `validation.detail` | 빈 값은 `null` |
| `stamp` | `created_at` | BSON datetime |

성공 메시지는 비어 있지 않은 JSON 객체 `command_json`이 필요하다.

### `TaskStatus`

| ROS 필드 | MongoDB 필드 |
| --- | --- |
| `state` | `current_state`, `status_history[].state` |
| `task_status` | `status` |
| `current_component`, `current_component_index` | `current_component.name`, `.index` |
| `inspection_result` | `final_inspection` |
| `stamp` | 시작·종료·이력 시각 |

검사 JSON은 `result`, `expected_counts`, `actual_counts`, `missing`,
`unexpected`, `detection_age`, `inspected_at`을 포함한다. 결과 범위, 객체와
배열 타입, 0 이상 감지 시간, timezone 포함 ISO 시각을 검증한다.

### `ComponentResult`

| ROS 필드 | MongoDB 필드 |
| --- | --- |
| `component` | `class_name` |
| `attempts_json` | `attempts` JSON 배열 |
| `error_code`, `detail` | `error.code`, `error.detail` |
| 나머지 필드 | 같은 이름 |

`attempt_count`는 배열 길이와 같아야 하고 `attempt_no`는 1부터 순차
증가해야 한다. `SUCCESS`의 마지막 Attempt는 `SUCCESS`여야 하며
`SKIPPED`의 배열은 비어 있어야 한다.

## 7. 재고 차감과 중복 방지

MongoDB에 최초 저장된 `SUCCESS` Component만 재고를 1개 차감한다.
저장 결과의 `upserted_id`로 신규 여부를 판정한다. `FAILED`, `SKIPPED`,
중복 메시지는 재고를 변경하지 않는다. 고유 인덱스를 사용하므로 노드 재시작
후에도 중복 차감을 방지한다.

## 8. 실행

환경변수는 `.env.example`을 기준으로 설정하고 `.env`는 커밋하지 않는다.
상세 DB 구축·초기화는 [`infra/README.md`](../infra/README.md)를 참고한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to kit_db
source install/setup.bash

docker compose up -d
docker compose ps postgres mongodb

set -a
source .env
set +a
ros2 run kit_db db_node
```

노드는 구독 전 두 DB에 ping한다. 어느 한쪽이 실패하면 구독하지 않고
종료한다. `Ctrl+C` 시 MongoDB client와 ROS lifecycle을 정리한다.

## 9. 테스트

```bash
python3 -m pytest -q \
  src/kit_db/test/test_db_node.py \
  src/kit_db/test/test_persistence.py \
  src/kit_db/test/test_inventory_deduction.py \
  src/kit_db/test/test_postgres.py

colcon test --packages-select kit_db
colcon test-result --verbose
```

실제 DB 통합 검증 기준:

- 최초 `SUCCESS`: MongoDB 문서 저장, 재고 1개 감소
- 동일 메시지 재발행: 문서와 재고 추가 변경 없음
- `FAILED`, `SKIPPED`: 문서 저장, 재고 유지
- 재고 0: 음수로 변경되지 않음

검증용 메시지 형식은 다음과 같다.

```bash
ros2 topic pub --once /kit/component_result \
  kit_interfaces/msg/ComponentResult \
  '{task_id: "TASK-MOCK-001", component_index: 0, component_total: 1,
    component: "분유", slot: "slot_1", status: "SUCCESS", attempt_count: 1,
    attempts_json: "[{\"attempt_no\":1,\"status\":\"SUCCESS\"}]",
    error_code: "", detail: "",
    started_at: {sec: 1788480000, nanosec: 0},
    ended_at: {sec: 1788480005, nanosec: 0}}'
```

## 10. 알려진 제한사항

MongoDB와 PostgreSQL 변경은 하나의 트랜잭션이 아니다. MongoDB에 신규
`SUCCESS`를 저장한 뒤 PostgreSQL 차감이 실패하면 기록은 존재하지만 재고는
변경되지 않을 수 있다. 동일 메시지는 중복으로 판정되어 차감을 자동
재시도하지 않는다. 현재 범위에서는 오류 로그 확인 후 수동 복구한다.

MongoDB는 Schema validator를 사용하지 않는다. 형식은 mapper와 이 문서의
계약으로 관리하며 DB에 직접 쓴 데이터에는 같은 검증이 적용되지 않는다.
