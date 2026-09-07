import rclpy
from rclpy.node import Node

from kit_interfaces.msg import (
    CommandResult,
    ComponentResult,
    TaskStatus,
)

from pymongo.errors import PyMongoError
from psycopg2 import Error as PostgreSQLError

from kit_db.config import MongoDBConfig, PostgreSQLConfig
from kit_db.postgres import PostgreSQL, InventoryRepository
from kit_db.mongodb import MongoDB, MongoRepository
from kit_db.persistence import PersistenceService

class DBNode(Node):
    def __init__(self, persistence, mongodb):
        super().__init__('kit_db')

        self.get_logger().info('DB node initialization started')

        # 데이터 영속성을 담당하는 객체
        self._persistence = persistence
        self._mongodb = mongodb

        self._command_subscription = self.create_subscription(
            CommandResult,
            '/kit/command_result',
            self._command_callback,
            10,
        )
        self.get_logger().info(
            'Subscribed to /kit/command_result (CommandResult)'
        )

        self._task_status_subscription = self.create_subscription(
            TaskStatus,
            '/kit/task_status',
            self._task_status_callback,
            10,
        )
        self.get_logger().info(
            'Subscribed to /kit/task_status (TaskStatus)'
        )

        self._component_subscription = self.create_subscription(
            ComponentResult,
            '/kit/component_result',
            self._component_callback,
            10,
        )
        self.get_logger().info(
            'Subscribed to /kit/component_result (ComponentResult)'
        )
        self.get_logger().info('DB node initialization completed')

    @staticmethod
    def _result_details(result):
        """Return useful, non-payload details from a database result."""
        if result is None:
            return 'result=None'

        details = []
        for attribute in (
            'acknowledged',
            'matched_count',
            'modified_count',
            'upserted_id',
        ):
            try:
                value = getattr(result, attribute)
            except (AttributeError, TypeError):
                continue

            details.append(f'{attribute}={value}')

        if details:
            return ', '.join(details)
        return f'result_type={type(result).__name__}'

    # 세 callback의 공통 실행 및 예외 처리 함수
    # -> 동일한 예외 처리 구조가 반복되는 것을 방지
    def _handle_message(self, message, handler, message_name):
        task_id = getattr(message, 'task_id', '<missing>')
        logger = self.get_logger()
        logger.info(
            f'Received {message_name}: task_id={task_id}'
        )

        try:
            logger.info(
                f'Starting persistence for {message_name}: '
                f'task_id={task_id}'
            )
            result = handler(message)
            logger.info(
                f'Completed persistence for {message_name}: '
                f'task_id={task_id}, {self._result_details(result)}'
            )
            return result
        # ValueError 예외 처리
        except ValueError as error:
            logger.error(
                f'Invalid {message_name} '
                f'for task {task_id}: {type(error).__name__}: {error}'
            )
        # MongoDB 관련 예외 처리
        except PyMongoError as error:
            logger.error(
                f'Failed to store {message_name} '
                f'for task {task_id}: {type(error).__name__}: {error}'
            )
        except PostgreSQLError as error:
            logger.error(
                f'Failed to update inventory for '
                f'{message_name} '
                f'for task {task_id}: {type(error).__name__}: {error}'
            )
        except Exception as error:
            logger.error(
                f'Unexpected failure while processing {message_name} '
                f'for task {task_id}: {type(error).__name__}: {error}'
            )
            raise

    def _command_callback(self, message):
        self._handle_message(
            message,
            self._persistence.record_command,
            'CommandResult',
        )


    def _task_status_callback(self, message):
        self._handle_message(
            message,
            self._persistence.record_task_status,
            'TaskStatus',
        )


    def _component_callback(self, message):
        self._handle_message(
            message,
            self._persistence.record_component,
            'ComponentResult',
        )

    def close(self):
        self.get_logger().info('Closing MongoDB connection')
        self._mongodb.close()
        self.get_logger().info('MongoDB connection closed')


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('kit_db.bootstrap')
    logger.info('DB node startup started')

    mongodb = None
    node = None

    try:
        # 환경변수에서 설정 로드
        logger.info('Loading database configuration from environment')
        Postgres_config = PostgreSQLConfig.from_environment()
        config = MongoDBConfig.from_environment()
        logger.info(
            'Database configuration loaded: '
            f'postgresql={Postgres_config.host}:'
            f'{Postgres_config.port}/{Postgres_config.database}, '
            f'mongodb={config.host}:{config.port}/{config.database}'
        )

        # PostgreSQL 생성 및 연결 확인
        logger.info('Creating PostgreSQL client')
        postgres = PostgreSQL(Postgres_config)
        logger.info('Checking PostgreSQL connection')
        postgres.ping()
        logger.info('PostgreSQL connection check succeeded')

        # MongoDB 생성 및 연결 확인
        logger.info('Creating MongoDB client')
        mongodb = MongoDB(config)
        logger.info('Checking MongoDB connection')
        mongodb.ping()
        logger.info('MongoDB connection check succeeded')

        # 의존성 조립
        logger.info('Creating database repositories')
        inventory_repository = InventoryRepository(postgres)
        mongo_repository = MongoRepository(mongodb)
        logger.info('Creating persistence service')
        persistence = PersistenceService(
            mongo_repository,
            inventory_repository,
        )

        # 생성된 객체를 DBNode에 전달
        node = DBNode(
            persistence=persistence,
            mongodb=mongodb,
        )

        logger.info('DB node is ready; starting ROS spin')
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info('DB node interrupted by user')
    except Exception as error:
        logger.error(
            'DB node startup or execution failed: '
            f'{type(error).__name__}: {error}'
        )
        raise

    finally:
        # 종료 처리
        logger.info('DB node shutdown started')
        if node is not None:
            node.close()
            node.destroy_node()
        elif mongodb is not None:
            mongodb.close()

        rclpy.shutdown()
        logger.info('DB node shutdown completed')
