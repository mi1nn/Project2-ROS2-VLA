"""Contract tests for component-driven inventory deduction."""

from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import psycopg2
import pytest

from kit_db.persistence import PersistenceService
from kit_db.postgres import InventoryRepository


@pytest.fixture
def postgres_context():
    database = Mock()
    connection_context = MagicMock()
    connection = connection_context.__enter__.return_value
    cursor_context = MagicMock()
    cursor = cursor_context.__enter__.return_value
    connection.cursor.return_value = cursor_context
    database.connect.return_value = connection_context

    return database, connection_context, cursor


def normalized_query(cursor):
    query = cursor.execute.call_args.args[0]
    return ' '.join(query.split())


@pytest.mark.parametrize(
    ('database_result', 'expected_result'),
    [
        ((7, 0, 'updated-at'), (7, 0, 'updated-at')),
        (None, None),
    ],
)
def test_inventory_decrement_is_atomic_and_never_goes_below_zero(
    postgres_context,
    database_result,
    expected_result,
):
    database, connection_context, cursor = postgres_context
    cursor.fetchone.return_value = database_result
    repository = InventoryRepository(database)

    result = repository.decrement('컵라면')

    query = normalized_query(cursor)
    assert 'SET quantity = inventory.quantity - 1' in query
    assert 'item.item_code = %s' in query
    assert 'inventory.quantity > 0' in query
    assert 'RETURNING inventory.item_id' in query
    cursor.execute.assert_called_once_with(
        cursor.execute.call_args.args[0],
        ('컵라면',),
    )
    cursor.fetchone.assert_called_once_with()
    connection_context.__exit__.assert_called_once_with(None, None, None)
    assert result == expected_result


def test_inventory_decrement_propagates_database_errors(
    postgres_context,
):
    database, connection_context, cursor = postgres_context
    error = psycopg2.OperationalError('database unavailable')
    cursor.execute.side_effect = error
    repository = InventoryRepository(database)

    with pytest.raises(
        psycopg2.OperationalError,
        match='database unavailable',
    ):
        repository.decrement('컵라면')

    exit_args = connection_context.__exit__.call_args.args
    assert exit_args[0] is psycopg2.OperationalError
    assert exit_args[1] is error


@pytest.fixture
def persistence_dependencies():
    mongo_repository = Mock(spec=['save_component'])
    inventory_repository = Mock(spec=['decrement'])
    service = PersistenceService(
        mongo_repository,
        inventory_repository,
    )

    return service, mongo_repository, inventory_repository


def component_document(status='SUCCESS'):
    return {
        'task_id': 'TASK-001',
        'component_index': 0,
        'class_name': '컵라면',
        'status': status,
    }


def mongo_result(upserted_id):
    return SimpleNamespace(upserted_id=upserted_id)


def test_first_successful_component_decrements_inventory_once(
    persistence_dependencies,
):
    service, mongo_repository, inventory_repository = (
        persistence_dependencies
    )
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    document = component_document()
    saved = mongo_result('mongo-id')
    mongo_repository.save_component.return_value = saved

    with patch(
        'kit_db.persistence.component_document',
        return_value=document,
    ):
        result = service.record_component(message)

    mongo_repository.save_component.assert_called_once_with(document)
    inventory_repository.decrement.assert_called_once_with('컵라면')
    assert result is saved


def test_duplicate_successful_component_does_not_decrement_inventory(
    persistence_dependencies,
):
    service, mongo_repository, inventory_repository = (
        persistence_dependencies
    )
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    mongo_repository.save_component.return_value = mongo_result(None)

    with patch(
        'kit_db.persistence.component_document',
        return_value=component_document(),
    ):
        service.record_component(message)

    inventory_repository.decrement.assert_not_called()


@pytest.mark.parametrize('status', ['FAILED', 'SKIPPED'])
def test_non_successful_component_does_not_decrement_inventory(
    persistence_dependencies,
    status,
):
    service, mongo_repository, inventory_repository = (
        persistence_dependencies
    )
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    mongo_repository.save_component.return_value = mongo_result('mongo-id')

    with patch(
        'kit_db.persistence.component_document',
        return_value=component_document(status),
    ):
        service.record_component(message)

    inventory_repository.decrement.assert_not_called()


def test_inventory_error_is_not_silently_ignored(
    persistence_dependencies,
):
    service, mongo_repository, inventory_repository = (
        persistence_dependencies
    )
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    mongo_repository.save_component.return_value = mongo_result('mongo-id')
    inventory_repository.decrement.side_effect = (
        psycopg2.OperationalError('database unavailable')
    )

    with (
        patch(
            'kit_db.persistence.component_document',
            return_value=component_document(),
        ),
        pytest.raises(
            psycopg2.OperationalError,
            match='database unavailable',
        ),
    ):
        service.record_component(message)


def test_mongodb_error_prevents_inventory_decrement(
    persistence_dependencies,
):
    service, mongo_repository, inventory_repository = (
        persistence_dependencies
    )
    message = SimpleNamespace(task_id='TASK-001', component_index=0)
    mongo_repository.save_component.side_effect = RuntimeError(
        'mongodb unavailable'
    )

    with (
        patch(
            'kit_db.persistence.component_document',
            return_value=component_document(),
        ),
        pytest.raises(RuntimeError, match='mongodb unavailable'),
    ):
        service.record_component(message)

    inventory_repository.decrement.assert_not_called()
