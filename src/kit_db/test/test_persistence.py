"""Tests for coordinating MongoDB writes and inventory updates."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from kit_db.persistence import PersistenceService


@pytest.fixture
def dependencies():
    collections = {
        'commands': Mock(),
        'kit_executions': Mock(),
        'component_executions': Mock(),
    }
    inventory = Mock()
    return collections, inventory


def test_command_is_stored_as_mapped_document(monkeypatch, dependencies):
    collections, inventory = dependencies
    message = SimpleNamespace(task_id='TASK-1')
    document = {'task_id': 'TASK-1', 'success': True}
    monkeypatch.setattr('kit_db.persistence.command_document', lambda _: document)

    PersistenceService(collections, inventory).record_command(message)

    collections['commands'].update_one.assert_called_once_with(
        {'task_id': 'TASK-1'}, {'$set': document}, upsert=True
    )


def test_task_status_is_upserted_by_task_id(monkeypatch, dependencies):
    collections, inventory = dependencies
    message = SimpleNamespace(task_id='TASK-1')
    update = {'$set': {'status': 'RUNNING'}}
    monkeypatch.setattr(
        'kit_db.persistence.task_status_update', lambda _: update
    )

    PersistenceService(collections, inventory).record_task_status(message)

    collections['kit_executions'].update_one.assert_called_once_with(
        {'task_id': 'TASK-1'}, update, upsert=True
    )


def test_component_is_upserted_by_composite_id(monkeypatch, dependencies):
    collections, inventory = dependencies
    message = SimpleNamespace(task_id='TASK-1', component_index=2)
    document = {
        'task_id': 'TASK-1',
        'component_index': 2,
        'class_name': 'mask',
        'status': 'FAILED',
        'attempts': [],
    }
    monkeypatch.setattr(
        'kit_db.persistence.component_document', lambda _: document
    )

    PersistenceService(collections, inventory).record_component(message)

    collections['component_executions'].update_one.assert_called_once_with(
        {'task_id': 'TASK-1', 'component_index': 2},
        {'$set': document},
        upsert=True,
    )


def test_successful_component_decrements_inventory(
    monkeypatch, dependencies
):
    collections, inventory = dependencies
    message = SimpleNamespace(task_id='TASK-1', component_index=0)
    document = {
        'task_id': 'TASK-1',
        'component_index': 0,
        'class_name': 'mask',
        'status': 'SUCCESS',
        'attempts': [{'result': 'SUCCESS'}],
    }
    monkeypatch.setattr(
        'kit_db.persistence.component_document', lambda _: document
    )

    PersistenceService(collections, inventory).record_component(message)

    inventory.decrement.assert_called_once_with('mask')


def test_failed_component_does_not_decrement_inventory(
    monkeypatch, dependencies
):
    collections, inventory = dependencies
    message = SimpleNamespace(task_id='TASK-1', component_index=0)
    document = {
        'task_id': 'TASK-1',
        'component_index': 0,
        'class_name': 'mask',
        'status': 'FAILED',
        'attempts': [{'result': 'FAILED'}],
    }
    monkeypatch.setattr(
        'kit_db.persistence.component_document', lambda _: document
    )

    PersistenceService(collections, inventory).record_component(message)

    inventory.decrement.assert_not_called()
