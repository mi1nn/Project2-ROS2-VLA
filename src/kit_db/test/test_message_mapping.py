"""Tests for converting ROS DB messages into MongoDB documents."""

from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from kit_db.message_mapping import (
    MessageValidationError,
    command_document,
    component_document,
    task_status_update,
)


def ros_time(seconds=0, nanoseconds=0):
    """Build a ROS-time-shaped test double."""
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def test_command_success_becomes_structured_document():
    message = SimpleNamespace(
        task_id='TASK-20260903-001',
        success=True,
        raw_text='지진 키트를 만들어줘',
        command_json='{"kit_type":"earthquake","items":[{"name":"mask","qty":1}]}',
        validation_result='VALID',
        error_code='',
        detail='',
        stamp=ros_time(1788398431, 421000000),
    )

    document = command_document(message)

    assert document == {
        'task_id': 'TASK-20260903-001',
        'success': True,
        'raw_text': '지진 키트를 만들어줘',
        'command_json': {
            'kit_type': 'earthquake',
            'items': [{'name': 'mask', 'qty': 1}],
        },
        'validation': {
            'result': 'VALID',
            'error_code': None,
            'detail': None,
        },
        'created_at': datetime(
            2026, 9, 3, 1, 20, 31, 421000, tzinfo=timezone.utc
        ),
    }


def test_failed_command_stores_no_command_json():
    message = SimpleNamespace(
        task_id='TASK-20260903-002',
        success=False,
        raw_text='없는 물건을 넣어줘',
        command_json='',
        validation_result='INVALID',
        error_code='unsupported_item',
        detail='지원하지 않는 품목',
        stamp=ros_time(1),
    )

    assert command_document(message)['command_json'] is None


def test_command_rejects_invalid_json_in_success_message():
    message = SimpleNamespace(
        task_id='TASK-1',
        success=True,
        raw_text='명령',
        command_json='{invalid',
        validation_result='VALID',
        error_code='',
        detail='',
        stamp=ros_time(1),
    )

    with pytest.raises(MessageValidationError, match='command_json'):
        command_document(message)


def test_task_status_builds_upsert_and_history_append():
    message = SimpleNamespace(
        task_id='TASK-20260903-001',
        state='EXECUTE',
        task_status='RUNNING',
        kit_type='earthquake',
        current_component='cup_ramen',
        current_component_index=1,
        component_total=3,
        inspection_result='',
        error_code='',
        detail='',
        stamp=ros_time(1788398431, 421000000),
    )

    update = task_status_update(message)

    assert update['$set']['status'] == 'RUNNING'
    assert update['$set']['current_state'] == 'EXECUTE'
    assert update['$set']['current_component'] == {
        'name': 'cup_ramen',
        'index': 1,
        'total': 3,
    }
    assert update['$set']['final_inspection'] is None
    assert update['$setOnInsert']['started_at'] == datetime(
        2026, 9, 3, 1, 20, 31, 421000, tzinfo=timezone.utc
    )
    assert update['$push']['status_history'] == {
        'state': 'EXECUTE',
        'status': 'RUNNING',
        'timestamp': datetime(
            2026, 9, 3, 1, 20, 31, 421000, tzinfo=timezone.utc
        ),
    }


def test_finished_task_parses_inspection_and_sets_end_time():
    message = SimpleNamespace(
        task_id='TASK-1',
        state='REPORT',
        task_status='SUCCESS',
        kit_type='earthquake',
        current_component='',
        current_component_index=0,
        component_total=1,
        inspection_result='{"result":"PASS","missing":[]}',
        error_code='',
        detail='',
        stamp=ros_time(2),
    )

    update = task_status_update(message)

    assert update['$set']['final_inspection'] == {
        'result': 'PASS',
        'missing': [],
    }
    assert update['$set']['ended_at'] == datetime.fromtimestamp(
        2, tz=timezone.utc
    )


def test_component_message_parses_attempts():
    message = SimpleNamespace(
        task_id='TASK-1',
        component_index=0,
        component_total=3,
        component='cup_ramen',
        slot='slot_1',
        status='SUCCESS',
        attempt_count=1,
        attempts_json='[{"attempt_no":1,"result":"SUCCESS"}]',
        error_code='',
        detail='',
        started_at=ros_time(1),
        ended_at=ros_time(2),
    )

    document = component_document(message)

    assert document['task_id'] == 'TASK-1'
    assert document['component_index'] == 0
    assert document['class_name'] == 'cup_ramen'
    assert document['attempts'] == [
        {'attempt_no': 1, 'result': 'SUCCESS'}
    ]
    assert document['error_code'] is None


@pytest.mark.parametrize('attempts_json', ['', '{}', '{invalid'])
def test_component_rejects_attempts_that_are_not_a_json_array(attempts_json):
    message = SimpleNamespace(
        task_id='TASK-1',
        component_index=0,
        component_total=1,
        component='mask',
        slot='slot_1',
        status='FAILED',
        attempt_count=1,
        attempts_json=attempts_json,
        error_code='empty_grasp',
        detail='',
        started_at=ros_time(1),
        ended_at=ros_time(2),
    )

    with pytest.raises(MessageValidationError, match='attempts_json'):
        component_document(message)
