"""Tests for MongoDB connection management."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from kit_db.mongodb import MongoDB

from pymongo.errors import ServerSelectionTimeoutError

import pytest


@pytest.fixture
def connection_info():
    """Return MongoDB settings without reading the process environment."""
    return SimpleNamespace(
        host='127.0.0.1',
        port=27017,
        database='kit_system',
        user='kit_admin',
        password='secret',
        authentication_database='admin',
    )


@patch('kit_db.mongodb.MongoClient')
def test_client_uses_connection_info(mock_client_class, connection_info):
    """The client must use credentials, auth DB, and bounded timeouts."""
    client = mock_client_class.return_value

    MongoDB(connection_info)

    mock_client_class.assert_called_once_with(
        host='127.0.0.1',
        port=27017,
        username='kit_admin',
        password='secret',
        authSource='admin',
        serverSelectionTimeoutMS=3000,
        connectTimeoutMS=3000,
    )
    client.__getitem__.assert_called_once_with('kit_system')


@patch('kit_db.mongodb.MongoClient')
def test_ping_checks_server_and_authentication(
    mock_client_class,
    connection_info,
):
    """Ping must perform a command against the admin database."""
    client = mock_client_class.return_value
    mongodb = MongoDB(connection_info)

    mongodb.ping()

    client.admin.command.assert_called_once_with('ping')


@patch('kit_db.mongodb.MongoClient')
def test_ping_propagates_connection_error(
    mock_client_class,
    connection_info,
):
    """Connection failures must be visible to the future DB node."""
    client = mock_client_class.return_value
    client.admin.command.side_effect = ServerSelectionTimeoutError(
        'unreachable'
    )
    mongodb = MongoDB(connection_info)

    with pytest.raises(ServerSelectionTimeoutError):
        mongodb.ping()


@patch('kit_db.mongodb.MongoClient')
def test_collection_uses_configured_database(
    mock_client_class,
    connection_info,
):
    """Collection lookup must stay within the configured application DB."""
    client = mock_client_class.return_value
    database = MagicMock()
    client.__getitem__.return_value = database
    collection = database.__getitem__.return_value
    mongodb = MongoDB(connection_info)

    result = mongodb.collection('commands')

    database.__getitem__.assert_called_once_with('commands')
    assert result is collection


@patch('kit_db.mongodb.MongoClient')
def test_close_releases_client(mock_client_class, connection_info):
    """Close must release resources owned by MongoClient."""
    client = mock_client_class.return_value
    mongodb = MongoDB(connection_info)

    mongodb.close()

    client.close.assert_called_once_with()
