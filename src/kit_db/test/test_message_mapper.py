"""Contract tests for converting ROS messages into MongoDB updates."""

from datetime import datetime, timezone
import json
from types import SimpleNamespace

import pytest

from kit_db.message_mapper import (
    command_document,
    component_document,
    task_status_update,
)


def ros_time(seconds=1725420612, nanoseconds=421000000):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def command_message(**overrides):
    values = {
        'task_id': 'TASK-001',
        'success': True,
        'raw_text': '컵라면 하나 담아줘',
        'command_json': json.dumps({
            'kit_type': 'earthquake',
            'items': [{'name': '컵라면', 'qty': 1}],
        }),
        'validation_result': 'VALID',
        'error_code': '',
        'detail': '',
        'stamp': ros_time(),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def task_status_message(**overrides):
    values = {
        'task_id': 'TASK-001',
        'state': 'EXECUTE',
        'task_status': 'RUNNING',
        'kit_type': 'earthquake',
        'current_component': '컵라면',
        'current_component_index': 0,
        'component_total': 1,
        'inspection_result': '',
        'error_code': '',
        'detail': '',
        'stamp': ros_time(),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def component_message(**overrides):
    attempts = [
        {
            'attempt_no': 1,
            'status': 'SUCCESS',
            'detection': {'score': 0.91},
        }
    ]
    values = {
        'task_id': 'TASK-001',
        'component_index': 0,
        'component_total': 1,
        'component': '컵라면',
        'slot': 'slot_1',
        'status': 'SUCCESS',
        'attempt_count': 1,
        'attempts_json': json.dumps(attempts),
        'error_code': '',
        'detail': '',
        'started_at': ros_time(),
        'ended_at': ros_time(1725420615, 182000000),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def expected_datetime(seconds=1725420612, microseconds=421000):
    return datetime.fromtimestamp(seconds, timezone.utc).replace(
        microsecond=microseconds
    )


def test_command_result_maps_to_nested_document():
    document = command_document(command_message())

    assert document == {
        'task_id': 'TASK-001',
        'raw_text': '컵라면 하나 담아줘',
        'command': {
            'kit_type': 'earthquake',
            'items': [{'name': '컵라면', 'qty': 1}],
        },
        'validation': {
            'success': True,
            'result': 'VALID',
            'error_code': None,
            'detail': None,
        },
        'created_at': expected_datetime(),
    }


def test_failed_command_allows_empty_command_as_null():
    document = command_document(command_message(
        success=False,
        command_json='',
        validation_result='INVALID',
        error_code='unsupported_item',
        detail='지원하지 않는 품목',
    ))

    assert document['command'] is None
    assert document['validation'] == {
        'success': False,
        'result': 'INVALID',
        'error_code': 'unsupported_item',
        'detail': '지원하지 않는 품목',
    }


@pytest.mark.parametrize('task_id', ['', '   '])
def test_command_rejects_blank_task_id(task_id):
    with pytest.raises(ValueError, match='task_id'):
        command_document(command_message(task_id=task_id))


@pytest.mark.parametrize('command_json', ['not-json', '[]', '"text"'])
def test_command_rejects_invalid_or_non_object_json(command_json):
    with pytest.raises(ValueError, match='command_json'):
        command_document(command_message(command_json=command_json))


def test_successful_command_rejects_empty_command_json():
    with pytest.raises(ValueError, match='command_json'):
        command_document(command_message(command_json=''))


def test_task_status_builds_idempotent_history_update():
    update = task_status_update(task_status_message())

    assert update == {
        '$setOnInsert': {
            'task_id': 'TASK-001',
            'started_at': expected_datetime(),
        },
        '$set': {
            'current_state': 'EXECUTE',
            'kit_type': 'earthquake',
            'status': 'RUNNING',
            'current_component': {
                'name': '컵라면',
                'index': 0,
            },
            'component_total': 1,
            'error': {
                'code': None,
                'detail': None,
            },
        },
        '$addToSet': {
            'status_history': {
                'state': 'EXECUTE',
                'timestamp': expected_datetime(),
            },
        },
    }


def test_terminal_task_maps_inspection_and_end_time():
    inspection = {
        'result': 'PASS',
        'expected_counts': {'컵라면': 1},
        'actual_counts': {'컵라면': 1},
        'missing': [],
        'unexpected': [],
        'detection_age': 0.15,
        'inspected_at': '2026-09-04T02:30:12.421Z',
    }
    update = task_status_update(task_status_message(
        state='REPORT',
        task_status='SUCCESS',
        inspection_result=json.dumps(inspection),
    ))

    assert 'started_at' not in update['$setOnInsert']
    assert update['$set']['ended_at'] == expected_datetime()
    assert update['$set']['final_inspection'] == {
        **inspection,
        'inspected_at': datetime(
            2026, 9, 4, 2, 30, 12, 421000, tzinfo=timezone.utc
        ),
    }


@pytest.mark.parametrize('task_id', ['', '   '])
def test_task_status_rejects_blank_task_id(task_id):
    with pytest.raises(ValueError, match='task_id'):
        task_status_update(task_status_message(task_id=task_id))


@pytest.mark.parametrize('inspection_result', ['not-json', '[]'])
def test_task_status_rejects_invalid_inspection_json(inspection_result):
    with pytest.raises(ValueError, match='inspection_result'):
        task_status_update(task_status_message(
            inspection_result=inspection_result
        ))


@pytest.mark.parametrize(
    ('field', 'invalid_value'),
    [
        ('result', 'UNKNOWN'),
        ('expected_counts', []),
        ('actual_counts', []),
        ('missing', {}),
        ('unexpected', {}),
        ('detection_age', -0.1),
        ('detection_age', True),
        ('inspected_at', 'not-a-time'),
    ],
)
def test_task_status_rejects_invalid_inspection_fields(
    field,
    invalid_value,
):
    inspection = {
        'result': 'PASS',
        'expected_counts': {'컵라면': 1},
        'actual_counts': {'컵라면': 1},
        'missing': [],
        'unexpected': [],
        'detection_age': 0.15,
        'inspected_at': '2026-09-04T02:30:12.421Z',
    }
    inspection[field] = invalid_value

    with pytest.raises(ValueError, match=field):
        task_status_update(task_status_message(
            inspection_result=json.dumps(inspection)
        ))


def test_component_result_maps_to_document():
    document = component_document(component_message())

    assert document == {
        'task_id': 'TASK-001',
        'component_index': 0,
        'component_total': 1,
        'class_name': '컵라면',
        'slot': 'slot_1',
        'status': 'SUCCESS',
        'attempt_count': 1,
        'attempts': [
            {
                'attempt_no': 1,
                'status': 'SUCCESS',
                'detection': {'score': 0.91},
            }
        ],
        'error': {
            'code': None,
            'detail': None,
        },
        'started_at': expected_datetime(),
        'ended_at': expected_datetime(1725420615, 182000),
    }


@pytest.mark.parametrize('task_id', ['', '   '])
def test_component_rejects_blank_task_id(task_id):
    with pytest.raises(ValueError, match='task_id'):
        component_document(component_message(task_id=task_id))


@pytest.mark.parametrize('attempts_json', ['not-json', '{}', 'null'])
def test_component_rejects_non_array_attempts(attempts_json):
    with pytest.raises(ValueError, match='attempts_json'):
        component_document(component_message(attempts_json=attempts_json))


def test_component_rejects_attempt_count_mismatch():
    with pytest.raises(ValueError, match='attempt_count'):
        component_document(component_message(attempt_count=2))


def test_component_rejects_non_sequential_attempt_numbers():
    attempts = [
        {'attempt_no': 1, 'status': 'FAILED'},
        {'attempt_no': 3, 'status': 'SUCCESS'},
    ]

    with pytest.raises(ValueError, match='attempt_no'):
        component_document(component_message(
            attempt_count=2,
            attempts_json=json.dumps(attempts),
        ))


def test_successful_component_requires_successful_last_attempt():
    attempts = [{'attempt_no': 1, 'status': 'FAILED'}]

    with pytest.raises(ValueError, match='last attempt'):
        component_document(component_message(
            attempts_json=json.dumps(attempts)
        ))


def test_skipped_component_accepts_no_attempts():
    document = component_document(component_message(
        status='SKIPPED',
        attempt_count=0,
        attempts_json='[]',
    ))

    assert document['attempts'] == []
    assert document['attempt_count'] == 0


@pytest.mark.parametrize('status', ['', 'RUNNING', 'UNKNOWN'])
def test_component_rejects_non_final_status(status):
    with pytest.raises(ValueError, match='status'):
        component_document(component_message(status=status))
