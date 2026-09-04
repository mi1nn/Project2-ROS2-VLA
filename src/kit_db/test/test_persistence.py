"""Contract tests for coordinating message mapping and MongoDB writes."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

from pymongo.errors import PyMongoError
import pytest

from kit_db.persistence import PersistenceService


@pytest.fixture
def repository():
    return Mock(spec=[
        'save_command',
        'update_task_status',
        'save_component',
    ])


@pytest.fixture
def service(repository):
    return PersistenceService(repository)


def test_record_command_maps_and_saves_document(service, repository):
    message = SimpleNamespace(task_id='TASK-001')
    document = {'task_id': 'TASK-001', 'command': {'items': []}}
    expected_result = Mock()
    repository.save_command.return_value = expected_result

    with patch(
        'kit_db.persistence.command_document',
        return_value=document,
    ) as mapper:
        result = service.record_command(message)

    mapper.assert_called_once_with(message)
    repository.save_command.assert_called_once_with(document)
    assert result is expected_result


def test_record_task_status_maps_and_updates_by_task_id(
    service,
    repository,
):
    message = SimpleNamespace(task_id='TASK-001')
    update = {
        '$setOnInsert': {'task_id': 'TASK-001'},
        '$set': {'status': 'RUNNING'},
    }
    expected_result = Mock()
    repository.update_task_status.return_value = expected_result

    with patch(
        'kit_db.persistence.task_status_update',
        return_value=update,
    ) as mapper:
        result = service.record_task_status(message)

    mapper.assert_called_once_with(message)
    repository.update_task_status.assert_called_once_with(
        'TASK-001',
        update,
    )
    assert result is expected_result


def test_record_component_maps_and_saves_document(service, repository):
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    document = {
        'task_id': 'TASK-001',
        'component_index': 0,
        'status': 'SUCCESS',
    }
    expected_result = Mock()
    repository.save_component.return_value = expected_result

    with patch(
        'kit_db.persistence.component_document',
        return_value=document,
    ) as mapper:
        result = service.record_component(message)

    mapper.assert_called_once_with(message)
    repository.save_component.assert_called_once_with(document)
    assert result is expected_result


@pytest.mark.parametrize(
    ('record_method', 'mapper_name', 'repository_method'),
    [
        ('record_command', 'command_document', 'save_command'),
        (
            'record_task_status',
            'task_status_update',
            'update_task_status',
        ),
        ('record_component', 'component_document', 'save_component'),
    ],
)
def test_mapper_validation_errors_are_propagated(
    service,
    repository,
    record_method,
    mapper_name,
    repository_method,
):
    message = SimpleNamespace(task_id='TASK-001')

    with patch(
        f'kit_db.persistence.{mapper_name}',
        side_effect=ValueError('invalid message'),
    ):
        with pytest.raises(ValueError, match='invalid message'):
            getattr(service, record_method)(message)

    getattr(repository, repository_method).assert_not_called()


@pytest.mark.parametrize(
    ('record_method', 'mapper_name', 'repository_method', 'mapped_value'),
    [
        (
            'record_command',
            'command_document',
            'save_command',
            {'task_id': 'TASK-001'},
        ),
        (
            'record_task_status',
            'task_status_update',
            'update_task_status',
            {'$set': {'status': 'RUNNING'}},
        ),
        (
            'record_component',
            'component_document',
            'save_component',
            {'task_id': 'TASK-001', 'component_index': 0},
        ),
    ],
)
def test_repository_errors_are_propagated(
    service,
    repository,
    record_method,
    mapper_name,
    repository_method,
    mapped_value,
):
    message = SimpleNamespace(task_id='TASK-001')
    getattr(repository, repository_method).side_effect = PyMongoError(
        'database unavailable'
    )

    with patch(
        f'kit_db.persistence.{mapper_name}',
        return_value=mapped_value,
    ):
        with pytest.raises(PyMongoError, match='database unavailable'):
            getattr(service, record_method)(message)
