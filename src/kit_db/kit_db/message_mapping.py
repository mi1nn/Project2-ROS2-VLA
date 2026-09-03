"""Convert ROS messages into MongoDB write models."""

import json
from datetime import datetime, timedelta, timezone

TERMINAL_TASK_STATUSES = frozenset({'SUCCESS', 'FAILED'})


class MessageValidationError(ValueError):
    """Raised when a DB-bound ROS message contains invalid data."""


def _optional_text(value):
    return value or None


def _datetime(ros_time):
    return (
        datetime.fromtimestamp(ros_time.sec, tz=timezone.utc)
        + timedelta(microseconds=ros_time.nanosec // 1000)
    )


def _json_value(raw_value, field_name, expected_type):
    try:
        value = json.loads(raw_value)
    except (TypeError, json.JSONDecodeError) as error:
        raise MessageValidationError(f'invalid {field_name}') from error

    if not isinstance(value, expected_type):
        raise MessageValidationError(f'invalid {field_name}')
    return value


def command_document(message):
    """Convert a CommandResult message into a commands document."""
    command = None
    if message.success:
        command = _json_value(message.command_json, 'command_json', dict)

    return {
        'task_id': message.task_id,
        'success': message.success,
        'raw_text': message.raw_text,
        'command_json': command,
        'validation': {
            'result': message.validation_result,
            'error_code': _optional_text(message.error_code),
            'detail': _optional_text(message.detail),
        },
        'created_at': _datetime(message.stamp),
    }


def task_status_update(message):
    """Convert a TaskStatus message into a kit_executions update."""
    timestamp = _datetime(message.stamp)
    inspection = None
    if message.inspection_result:
        inspection = _json_value(
            message.inspection_result, 'inspection_result', dict
        )

    set_values = {
        'task_id': message.task_id,
        'kit_type': message.kit_type,
        'status': message.task_status,
        'current_state': message.state,
        'current_component': {
            'name': message.current_component,
            'index': message.current_component_index,
            'total': message.component_total,
        },
        'final_inspection': inspection,
        'error_code': _optional_text(message.error_code),
        'detail': _optional_text(message.detail),
    }
    if message.task_status in TERMINAL_TASK_STATUSES:
        set_values['ended_at'] = timestamp

    return {
        '$set': set_values,
        '$setOnInsert': {'started_at': timestamp},
        '$push': {
            'status_history': {
                'state': message.state,
                'status': message.task_status,
                'timestamp': timestamp,
            }
        },
    }


def component_document(message):
    """Convert a ComponentResult message into a component document."""
    attempts = _json_value(message.attempts_json, 'attempts_json', list)

    return {
        'task_id': message.task_id,
        'component_index': message.component_index,
        'component_total': message.component_total,
        'class_name': message.component,
        'slot': message.slot,
        'status': message.status,
        'attempt_count': message.attempt_count,
        'attempts': attempts,
        'error_code': _optional_text(message.error_code),
        'detail': _optional_text(message.detail),
        'started_at': _datetime(message.started_at),
        'ended_at': _datetime(message.ended_at),
    }
