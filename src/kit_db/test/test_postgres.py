"""Contract tests for PostgreSQL connection and inventory queries."""

from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import psycopg2
import pytest

from kit_db.postgres import InventoryRepository, PostgreSQL


@pytest.fixture
def connection_info():
    return SimpleNamespace(
        host='127.0.0.1',
        port=5432,
        database='kit_system',
        user='kit_app',
        password='secret',
    )


@pytest.fixture
def database_context():
    database = Mock()
    connection_context = MagicMock()
    connection = connection_context.__enter__.return_value
    cursor_context = MagicMock()
    cursor = cursor_context.__enter__.return_value
    connection.cursor.return_value = cursor_context
    database.connect.return_value = connection_context
    return database, connection_context, connection, cursor


def normalized_query(cursor):
    query = cursor.execute.call_args.args[0]
    return ' '.join(query.split())


@patch('kit_db.postgres.psycopg2.connect')
def test_connect_uses_configuration_and_timeout(
    connect,
    connection_info,
):
    database = PostgreSQL(connection_info)

    result = database.connect()

    connect.assert_called_once_with(
        host='127.0.0.1',
        port=5432,
        dbname='kit_system',
        user='kit_app',
        password='secret',
        connect_timeout=3,
    )
    assert result is connect.return_value


@patch('kit_db.postgres.psycopg2.connect')
def test_connect_propagates_database_error(connect, connection_info):
    connect.side_effect = psycopg2.OperationalError('connection failed')
    database = PostgreSQL(connection_info)

    with pytest.raises(psycopg2.OperationalError, match='connection failed'):
        database.connect()


def test_ping_executes_select_one(connection_info):
    database = PostgreSQL(connection_info)
    connection_context = MagicMock()
    connection = connection_context.__enter__.return_value
    cursor_context = MagicMock()
    cursor = cursor_context.__enter__.return_value
    cursor.fetchone.return_value = (1,)
    connection.cursor.return_value = cursor_context

    with patch.object(
        database,
        'connect',
        return_value=connection_context,
    ):
        result = database.ping()

    cursor.execute.assert_called_once_with('SELECT 1')
    cursor.fetchone.assert_called_once_with()
    assert result == (1,)
    connection_context.__exit__.assert_called_once()
    cursor_context.__exit__.assert_called_once()


def test_find_all_returns_inventory_rows(database_context):
    database, connection_context, _, cursor = database_context
    rows = [
        (0, '마스크', '마스크', 2, 'updated-at'),
        (1, '분유', '분유', 20, 'updated-at'),
    ]
    cursor.fetchall.return_value = rows
    repository = InventoryRepository(database)

    result = repository.find_all()

    query = normalized_query(cursor)
    assert 'FROM item AS i' in query
    assert 'JOIN inventory AS v USING (item_id)' in query
    assert 'ORDER BY i.item_id' in query
    cursor.execute.assert_called_once()
    cursor.fetchall.assert_called_once_with()
    assert result == rows
    connection_context.__exit__.assert_called_once()


@pytest.mark.parametrize(
    ('database_result', 'expected_result'),
    [
        ((7, 0, 'updated-at'), (7, 0, 'updated-at')),
        (None, None),
    ],
)
def test_decrement_is_atomic_and_returns_database_result(
    database_context,
    database_result,
    expected_result,
):
    database, connection_context, _, cursor = database_context
    cursor.fetchone.return_value = database_result
    repository = InventoryRepository(database)

    result = repository.decrement('컵라면')

    query = normalized_query(cursor)
    assert 'SET quantity = inventory.quantity - 1' in query
    assert 'updated_at = NOW()' in query
    assert 'item.item_code = %s' in query
    assert 'inventory.quantity > 0' in query
    assert 'RETURNING inventory.item_id' in query
    cursor.execute.assert_called_once_with(
        cursor.execute.call_args.args[0],
        ('컵라면',),
    )
    cursor.fetchone.assert_called_once_with()
    assert result == expected_result
    connection_context.__exit__.assert_called_once()


@pytest.mark.parametrize('method_name', ['find_all', 'decrement'])
def test_inventory_repository_propagates_database_error(
    method_name,
):
    database = Mock()
    database.connect.side_effect = psycopg2.OperationalError(
        'database unavailable'
    )
    repository = InventoryRepository(database)

    with pytest.raises(
        psycopg2.OperationalError,
        match='database unavailable',
    ):
        method = getattr(repository, method_name)
        method('컵라면') if method_name == 'decrement' else method()
