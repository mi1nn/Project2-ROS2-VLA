"""Contract tests for the ROS DB node."""

from types import SimpleNamespace
from unittest.mock import call, Mock, patch

from kit_interfaces.msg import CommandResult, ComponentResult, TaskStatus
from pymongo.errors import PyMongoError
from psycopg2 import OperationalError
import pytest
from rclpy.node import Node

from kit_db.db_node import DBNode, main


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


def test_component_callback_isolates_postgresql_failures(
    node_dependencies,
):
    node, persistence, _, logger, _ = node_dependencies
    message = SimpleNamespace(task_id='TASK-001')
    persistence.record_component.side_effect = OperationalError(
        'inventory database unavailable'
    )

    node._component_callback(message)

    logger.error.assert_called_once()
    assert 'TASK-001' in logger.error.call_args.args[0]
    assert 'inventory database unavailable' in logger.error.call_args.args[0]


def test_close_releases_mongodb_connection(node_dependencies):
    node, _, mongodb, _, _ = node_dependencies

    node.close()

    mongodb.close.assert_called_once_with()


@pytest.fixture
def lifecycle_dependencies():
    with (
        patch('kit_db.db_node.rclpy.init') as init,
        patch('kit_db.db_node.rclpy.spin') as spin,
        patch('kit_db.db_node.rclpy.shutdown') as shutdown,
        patch(
            'kit_db.db_node.MongoDBConfig.from_environment'
        ) as load_mongodb_config,
        patch(
            'kit_db.db_node.PostgreSQLConfig.from_environment'
        ) as load_postgresql_config,
        patch('kit_db.db_node.MongoDB') as mongodb_class,
        patch('kit_db.db_node.PostgreSQL') as postgresql_class,
        patch(
            'kit_db.db_node.MongoRepository'
        ) as mongo_repository_class,
        patch(
            'kit_db.db_node.InventoryRepository'
        ) as inventory_repository_class,
        patch('kit_db.db_node.PersistenceService') as service_class,
        patch('kit_db.db_node.DBNode') as node_class,
    ):
        yield SimpleNamespace(
            init=init,
            spin=spin,
            shutdown=shutdown,
            load_mongodb_config=load_mongodb_config,
            load_postgresql_config=load_postgresql_config,
            mongodb_class=mongodb_class,
            postgresql_class=postgresql_class,
            mongo_repository_class=mongo_repository_class,
            inventory_repository_class=inventory_repository_class,
            service_class=service_class,
            node_class=node_class,
        )


def test_main_wires_dependencies_and_spins(lifecycle_dependencies):
    dependencies = lifecycle_dependencies
    mongodb_config = Mock()
    postgresql_config = Mock()
    dependencies.load_mongodb_config.return_value = mongodb_config
    dependencies.load_postgresql_config.return_value = postgresql_config
    mongodb = dependencies.mongodb_class.return_value
    postgresql = dependencies.postgresql_class.return_value
    mongo_repository = (
        dependencies.mongo_repository_class.return_value
    )
    inventory_repository = (
        dependencies.inventory_repository_class.return_value
    )
    persistence = dependencies.service_class.return_value
    node = dependencies.node_class.return_value

    main(args=['--ros-args'])

    dependencies.init.assert_called_once_with(args=['--ros-args'])
    dependencies.load_mongodb_config.assert_called_once_with()
    dependencies.load_postgresql_config.assert_called_once_with()
    dependencies.mongodb_class.assert_called_once_with(mongodb_config)
    dependencies.postgresql_class.assert_called_once_with(
        postgresql_config
    )
    mongodb.ping.assert_called_once_with()
    postgresql.ping.assert_called_once_with()
    dependencies.mongo_repository_class.assert_called_once_with(mongodb)
    dependencies.inventory_repository_class.assert_called_once_with(
        postgresql
    )
    dependencies.service_class.assert_called_once_with(
        mongo_repository,
        inventory_repository,
    )
    dependencies.node_class.assert_called_once_with(
        persistence=persistence,
        mongodb=mongodb,
    )
    dependencies.spin.assert_called_once_with(node)
    node.close.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    dependencies.shutdown.assert_called_once_with()


def test_main_cleans_up_after_keyboard_interrupt(lifecycle_dependencies):
    dependencies = lifecycle_dependencies
    mongodb = dependencies.mongodb_class.return_value
    postgresql = dependencies.postgresql_class.return_value
    node = dependencies.node_class.return_value
    dependencies.spin.side_effect = KeyboardInterrupt

    main()

    postgresql.ping.assert_called_once_with()
    mongodb.ping.assert_called_once_with()
    node.close.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    dependencies.shutdown.assert_called_once_with()


def test_main_closes_mongodb_when_ping_fails(lifecycle_dependencies):
    dependencies = lifecycle_dependencies
    mongodb = dependencies.mongodb_class.return_value
    mongodb.ping.side_effect = PyMongoError('connection failed')

    with pytest.raises(PyMongoError, match='connection failed'):
        main()

    mongodb.close.assert_called_once_with()
    dependencies.mongo_repository_class.assert_not_called()
    dependencies.inventory_repository_class.assert_not_called()
    dependencies.service_class.assert_not_called()
    dependencies.node_class.assert_not_called()
    dependencies.spin.assert_not_called()
    dependencies.shutdown.assert_called_once_with()


def test_main_stops_before_mongodb_when_postgresql_ping_fails(
    lifecycle_dependencies,
):
    dependencies = lifecycle_dependencies
    postgresql = dependencies.postgresql_class.return_value
    postgresql.ping.side_effect = OperationalError('connection failed')

    with pytest.raises(OperationalError, match='connection failed'):
        main()

    dependencies.mongodb_class.assert_not_called()
    dependencies.mongo_repository_class.assert_not_called()
    dependencies.inventory_repository_class.assert_not_called()
    dependencies.service_class.assert_not_called()
    dependencies.node_class.assert_not_called()
    dependencies.spin.assert_not_called()
    dependencies.shutdown.assert_called_once_with()
