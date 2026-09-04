"""Contract tests for the ROS DB node."""

from types import SimpleNamespace
from unittest.mock import call, Mock, patch

from kit_interfaces.msg import CommandResult, ComponentResult, TaskStatus
from pymongo.errors import PyMongoError
import pytest
from rclpy.node import Node

from kit_db.db_node import DBNode


@pytest.fixture
def node_dependencies():
    persistence = Mock(spec=[
        'record_command',
        'record_task_status',
        'record_component',
    ])
    mongodb = Mock(spec=['close'])
    logger = Mock(spec=['info', 'warning', 'error'])

    with (
        patch.object(Node, '__init__', return_value=None),
        patch.object(Node, 'create_subscription') as create_subscription,
        patch.object(Node, 'get_logger', return_value=logger),
    ):
        node = DBNode(persistence, mongodb)
        yield node, persistence, mongodb, logger, create_subscription


def test_node_subscribes_to_all_database_topics(node_dependencies):
    node, _, _, _, create_subscription = node_dependencies

    assert create_subscription.call_args_list == [
        call(
            CommandResult,
            '/kit/command_result',
            node._command_callback,
            10,
        ),
        call(
            TaskStatus,
            '/kit/task_status',
            node._task_status_callback,
            10,
        ),
        call(
            ComponentResult,
            '/kit/component_result',
            node._component_callback,
            10,
        ),
    ]


@pytest.mark.parametrize(
    ('callback_name', 'service_method'),
    [
        ('_command_callback', 'record_command'),
        ('_task_status_callback', 'record_task_status'),
        ('_component_callback', 'record_component'),
    ],
)
def test_callbacks_route_messages_to_persistence_service(
    node_dependencies,
    callback_name,
    service_method,
):
    node, persistence, _, _, _ = node_dependencies
    message = SimpleNamespace(task_id='TASK-001')

    getattr(node, callback_name)(message)

    getattr(persistence, service_method).assert_called_once_with(message)


@pytest.mark.parametrize(
    ('callback_name', 'service_method'),
    [
        ('_command_callback', 'record_command'),
        ('_task_status_callback', 'record_task_status'),
        ('_component_callback', 'record_component'),
    ],
)
def test_callbacks_reject_invalid_messages_without_raising(
    node_dependencies,
    callback_name,
    service_method,
):
    node, persistence, _, logger, _ = node_dependencies
    message = SimpleNamespace(task_id='TASK-001')
    getattr(persistence, service_method).side_effect = ValueError(
        'invalid message'
    )

    getattr(node, callback_name)(message)

    logger.error.assert_called_once()
    assert 'TASK-001' in logger.error.call_args.args[0]
    assert 'invalid message' in logger.error.call_args.args[0]


@pytest.mark.parametrize(
    ('callback_name', 'service_method'),
    [
        ('_command_callback', 'record_command'),
        ('_task_status_callback', 'record_task_status'),
        ('_component_callback', 'record_component'),
    ],
)
def test_callbacks_isolate_mongodb_failures(
    node_dependencies,
    callback_name,
    service_method,
):
    node, persistence, _, logger, _ = node_dependencies
    message = SimpleNamespace(task_id='TASK-001')
    getattr(persistence, service_method).side_effect = PyMongoError(
        'database unavailable'
    )

    getattr(node, callback_name)(message)

    logger.error.assert_called_once()
    assert 'TASK-001' in logger.error.call_args.args[0]
    assert 'database unavailable' in logger.error.call_args.args[0]


def test_close_releases_mongodb_connection(node_dependencies):
    node, _, mongodb, _, _ = node_dependencies

    node.close()

    mongodb.close.assert_called_once_with()
