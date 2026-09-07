"""Tests for persisting mapped documents in MongoDB."""

from unittest.mock import call, Mock

import pytest

from kit_db.mongodb import MongoRepository


@pytest.fixture
def repository():
    mongodb = Mock()
    collections = {
        'commands': Mock(),
        'kit_executions': Mock(),
        'component_executions': Mock(),
    }
    mongodb.collection.side_effect = collections.__getitem__
    return MongoRepository(mongodb), collections, mongodb


def test_repository_selects_contract_collections(repository):
    mongo_repository, _, mongodb = repository

    assert mongo_repository is not None
    assert mongodb.collection.call_args_list == [
        call('commands'),
        call('kit_executions'),
        call('component_executions'),
    ]


def test_save_command_inserts_once_without_overwriting(repository):
    mongo_repository, collections, _ = repository
    document = {
        'task_id': 'TASK-001',
        'raw_text': '컵라면 하나 담아줘',
    }
    expected_result = Mock()
    collections['commands'].update_one.return_value = expected_result

    result = mongo_repository.save_command(document)

    collections['commands'].update_one.assert_called_once_with(
        {'task_id': 'TASK-001'},
        {'$setOnInsert': document},
        upsert=True,
    )
    assert result is expected_result


def test_update_task_status_updates_by_task_id(repository):
    mongo_repository, collections, _ = repository
    update = {
        '$setOnInsert': {'task_id': 'TASK-001'},
        '$set': {'status': 'RUNNING'},
        '$addToSet': {
            'status_history': {
                'state': 'EXECUTE',
                'timestamp': 'timestamp',
            },
        },
    }
    expected_result = Mock()
    collections['kit_executions'].update_one.return_value = expected_result

    result = mongo_repository.update_task_status('TASK-001', update)

    collections['kit_executions'].update_one.assert_called_once_with(
        {'task_id': 'TASK-001'},
        update,
        upsert=True,
    )
    assert result is expected_result


def test_save_component_inserts_once_by_composite_identity(repository):
    mongo_repository, collections, _ = repository
    document = {
        'task_id': 'TASK-001',
        'component_index': 0,
        'status': 'SUCCESS',
    }
    expected_result = Mock()
    collection = collections['component_executions']
    collection.update_one.return_value = expected_result

    result = mongo_repository.save_component(document)

    collection.update_one.assert_called_once_with(
        {
            'task_id': 'TASK-001',
            'component_index': 0,
        },
        {'$setOnInsert': document},
        upsert=True,
    )
    assert result is expected_result
