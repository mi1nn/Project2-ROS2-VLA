# ROS 메시지를 MongoDB에 저장 가능한 Python dictionary로 변환하는 모듈

import json
from datetime import datetime, timedelta, timezone


# 문자열 검사
def _required_text(value, field_name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{field_name} is required')
    return value

# 빈 문자열을 None으로 변환 -> error_code, detail에서 활용
def _empty_to_none(value):
    if value == '':
        return None
    return value

# ROS Time 변환
def _ros_time_to_datetime(value):
    return (
        datetime.fromtimestamp(value.sec, timezone.utc)
        + timedelta(microseconds=value.nanosec // 1000)
    )

# JSON 파싱
def _parse_json(value, field_name):
    try:
        return json.loads(value)
    except (json.JSONDecodeError, TypeError) as error:
        raise ValueError(f'{field_name} must be valid JSON') from error


def _parse_json_object(value, field_name):
    parsed = _parse_json(value, field_name)

    if not isinstance(parsed, dict):
        raise ValueError(f'{field_name} must be a JSON object')

    return parsed


def _parse_json_array(value, field_name):
    parsed = _parse_json(value, field_name)

    if not isinstance(parsed, list):
        raise ValueError(f'{field_name} must be a JSON array')

    return parsed

# =====================================================================
# CommandResult
# =====================================================================

# CommandResult 메시지를 commands 컬렉션에 저장 가능한 Python dictionary로 변환
def command_document(message):
    task_id = _required_text(message.task_id, 'task_id')

    if message.command_json:
        command = _parse_json_object(
            message.command_json,
            'command_json',
        )
    elif message.success:
        raise ValueError(
            'command_json is required for a successful command'
        )
    else:
        command = None

    return {
        'task_id': task_id,
        'raw_text': message.raw_text,
        'command': command,
        'validation': {
            'success': message.success,
            'result': message.validation_result,
            'error_code': _empty_to_none(message.error_code),
            'detail': _empty_to_none(message.detail),
        },
        'created_at': _ros_time_to_datetime(message.stamp),
    }

# 검사 시각 변환
def _iso_datetime(value, field_name):
    if not isinstance(value, str):
        raise ValueError(f'{field_name} must be an ISO datetime')

    try:
        parsed = datetime.fromisoformat(
            value.replace('Z', '+00:00')
        )
    except ValueError as error:
        raise ValueError(
            f'{field_name} must be an ISO datetime'
        ) from error

    if parsed.tzinfo is None:
        raise ValueError(f'{field_name} must include a timezone')

    return parsed.astimezone(timezone.utc)


# =====================================================================
# TaskStatus
# =====================================================================

# final_inspection 검증
INSPECTION_RESULTS = {'PASS', 'FAIL', 'ERROR'}

def _validate_inspection(inspection_result):
    if inspection_result.get('result') not in INSPECTION_RESULTS:
        raise ValueError(
            'inspection_result.result must be PASS, FAIL, or ERROR'
        )

    if not isinstance(inspection_result.get('expected_counts'), dict):
        raise ValueError(
            'inspection_result.expected_counts must be an object'
        )

    actual_counts = inspection_result.get('actual_counts')
    if actual_counts is not None and not isinstance(
        actual_counts, dict
    ):
        raise ValueError(
            'inspection_result.actual_counts must be an object or null'
        )

    if not isinstance(inspection_result.get('missing'), list):
        raise ValueError(
            'inspection_result.missing must be an array'
        )

    if not isinstance(inspection_result.get('unexpected'), list):
        raise ValueError(
            'inspection_result.unexpected must be an array'
        )

    detection_age = inspection_result.get('detection_age')
    if (
        detection_age is not None
        and (
            isinstance(detection_age, bool)
            or not isinstance(detection_age, (int, float))
            or detection_age < 0
        )
    ):
        raise ValueError(
            'inspection_result.detection_age '
            'must be a non-negative number or null'
        )

    inspection_result['inspected_at'] = _iso_datetime(
        inspection_result.get('inspected_at'),
        'inspection_result.inspected_at',
    )

    return inspection_result

# TaskStatus 메시지를 tasks 컬렉션에 저장 가능한 Python dictionary로 변환
# update_one()에 전달할 update 객체를 반환
def task_status_update(message):
    task_id = _required_text(message.task_id, 'task_id')
    timestamp = _ros_time_to_datetime(message.stamp)

    set_on_insert = {
        'task_id': task_id,
    }

    if message.task_status == 'RUNNING':
        set_on_insert['started_at'] = timestamp

    set_values = {
        'current_state': message.state,
        'kit_type': message.kit_type,
        'status': message.task_status,
        'current_component': {
            'name': message.current_component,
            'index': message.current_component_index,
        },
        'component_total': message.component_total,
        'error': {
            'code': _empty_to_none(message.error_code),
            'detail': _empty_to_none(message.detail),
        },
    }

    if message.inspection_result:
        inspection = _parse_json_object(
            message.inspection_result,
            'inspection_result',
        )
        set_values['final_inspection'] = (
            _validate_inspection(inspection)
        )

    if message.task_status in {'SUCCESS', 'FAILED'}:
        set_values['ended_at'] = timestamp

    return {
        '$setOnInsert': set_on_insert,
        '$set': set_values,
        '$addToSet': {
            'status_history': {
                'state': message.state,
                'timestamp': timestamp,
            },
        },
    }


# =====================================================================
# ComponentResult
# =====================================================================

# Component 상태 검증
COMPONENT_FINAL_STATUSES = {
    'SUCCESS',
    'FAILED',
    'SKIPPED',
}

def _validate_component_status(status):
    if status not in COMPONENT_FINAL_STATUSES:
        raise ValueError(
            'status must be SUCCESS, FAILED, or SKIPPED'
        )

def _validate_attempts(status, attempt_count, attempts):
    if attempt_count != len(attempts):
        raise ValueError(
            'attempt_count must match attempts length'
        )

    for expected_number, attempt in enumerate(attempts, start=1):
        if not isinstance(attempt, dict):
            raise ValueError(
                'attempts_json entries must be objects'
            )

        if attempt.get('attempt_no') != expected_number:
            raise ValueError(
                'attempt_no must start at 1 and be sequential'
            )

    if status == 'SUCCESS':
        if not attempts:
            raise ValueError(
                'successful component requires a last attempt'
            )

        if attempts[-1].get('status') != 'SUCCESS':
            raise ValueError(
                'successful component requires '
                'a successful last attempt'
            )

    if status == 'SKIPPED' and attempts:
        raise ValueError(
            'skipped component must not contain attempts'
        )

# TaskStatus 메시지를 tasks 컬렉션에 저장 가능한 Python dictionary로 변환
def component_document(message):
    task_id = _required_text(message.task_id, 'task_id')
    _validate_component_status(message.status)

    attempts = _parse_json_array(
        message.attempts_json,
        'attempts_json',
    )

    _validate_attempts(
        message.status,
        message.attempt_count,
        attempts,
    )

    return {
        'task_id': task_id,
        'component_index': message.component_index,
        'component_total': message.component_total,
        'class_name': message.component,
        'slot': message.slot,
        'status': message.status,
        'attempt_count': message.attempt_count,
        'attempts': attempts,
        'error': {
            'code': _empty_to_none(message.error_code),
            'detail': _empty_to_none(message.detail),
        },
        'started_at': _ros_time_to_datetime(
            message.started_at
        ),
        'ended_at': _ros_time_to_datetime(
            message.ended_at
        ),
    }
