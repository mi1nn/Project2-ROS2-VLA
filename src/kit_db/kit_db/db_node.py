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

        # 데이터 영속성을 담당하는 객체
        self._persistence = persistence
        self._mongodb = mongodb

        self._command_subscription = self.create_subscription(
            CommandResult,
            '/kit/command_result',
            self._command_callback,
            10,
        )

        self._task_status_subscription = self.create_subscription(
            TaskStatus,
            '/kit/task_status',
            self._task_status_callback,
            10,
        )

        self._component_subscription = self.create_subscription(
            ComponentResult,
            '/kit/component_result',
            self._component_callback,
            10,
        )

    # 세 callback의 공통 실행 및 예외 처리 함수
    # -> 동일한 예외 처리 구조가 반복되는 것을 방지
    def _handle_message(self, message, handler, message_name):
        try:
            handler(message)
        # ValueError 예외 처리
        except ValueError as error:
            self.get_logger().error(
                f'Invalid {message_name} '
                f'for task {message.task_id}: {error}'
            )
        # MongoDB 관련 예외 처리
        except PyMongoError as error:
            self.get_logger().error(
                f'Failed to store {message_name} '
                f'for task {message.task_id}: {error}'
            )
        except PostgreSQLError as error:
            self.get_logger().error(
                f'Failed to update inventory for '
                f'{message_name} '
                f'for task {message.task_id}: {error}'
            )

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
        self._mongodb.close()


def main(args=None):
    rclpy.init(args=args)

    mongodb = None
    node = None

    try:
        # 환경변수에서 설정 로드
        Postgres_config = PostgreSQLConfig.from_environment()
        config = MongoDBConfig.from_environment()

        # PostgreSQL 생성 및 연결 확인
        postgres = PostgreSQL(Postgres_config)
        postgres.ping()

        # MongoDB 생성 및 연결 확인
        mongodb = MongoDB(config)
        mongodb.ping()

        # 의존성 조립
        inventory_repository = InventoryRepository(postgres)
        mongo_repository = MongoRepository(mongodb)
        persistence = PersistenceService(
            mongo_repository,
            inventory_repository,
        )

        # 생성된 객체를 DBNode에 전달
        node = DBNode(
            persistence=persistence,
            mongodb=mongodb,
        )
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 종료 처리
        if node is not None:
            node.close()
            node.destroy_node()
        elif mongodb is not None:
            mongodb.close()

        rclpy.shutdown()