import rclpy
from rclpy.node import Node

import json
import uuid

from kit_interfaces.msg import (
    CommandResult,
    ComponentResult,
    TaskStatus,
)

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')

        self.get_logger().info('Mock Publishing')

        self.command_result_pub = self.create_publisher(
            CommandResult,
            "/kit/command_result",
            10,
        )

        self.task_status_pub = self.create_publisher(
            TaskStatus, "/kit/task_status", 10
        )
        
        # 품목 최종 결과 토픽: 재시도 내역은 Attempt 배열로 한 메시지에 담는다.
        self.component_result_pub = self.create_publisher(
            ComponentResult, "/kit/component_result", 10
        )

        self.phase = 0
        self.run_count = 0
        self.task_id = None

        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        if self.phase == 0:
            # 반복마다 반드시 새로운 ID 생성
            self.task_id = (
                f'TASK-MOCK-{self.run_count:06d}-'
                f'{uuid.uuid4().hex[:8]}'
            )
            self.publish_command_result()
            step_name = 'CommandResult'

        elif self.phase == 1:
            self.publish_task_status('RUNNING')
            step_name = 'TaskStatus(RUNNING)'

        elif self.phase == 2:
            self.publish_component_result()
            step_name = 'ComponentResult'

        else:
            self.publish_task_status('SUCCESS')
            step_name = 'TaskStatus(SUCCESS)'

        self.get_logger().info(
            f'Published {step_name}: '
            f'run={self.run_count}, task_id={self.task_id}'
        )

        self.phase += 1

        if self.phase == 4:
            self.phase = 0
            self.run_count += 1


    def publish_command_result(self):
        message = CommandResult()
        message.task_id = self.task_id
        message.success = True
        message.raw_text = '분유를 담아줘'
        message.command_json = json.dumps(
            {
                'kit_type': 'test',
                'items': [
                    {
                        'component': '분유',
                        'quantity': 1,
                    },
                ],
            },
            ensure_ascii=False,
        )
        message.validation_result = 'VALID'
        message.error_code = ''
        message.detail = ''
        message.stamp = self.get_clock().now().to_msg()

        self.command_result_pub.publish(message)


    def publish_task_status(self, status):
        message = TaskStatus()
        message.task_id = self.task_id
        message.state = (
            'EXECUTE' if status == 'RUNNING' else 'COMPLETE'
        )
        message.task_status = status
        message.kit_type = 'test'
        message.current_component = '분유'
        message.current_component_index = 0
        message.component_total = 1
        message.inspection_result = ''
        message.error_code = ''
        message.detail = ''
        message.stamp = self.get_clock().now().to_msg()

        self.task_status_pub.publish(message)

    def publish_component_result(self):
        message = ComponentResult()
        message.task_id = self.task_id
        message.component_index = 0
        message.component_total = 1
        message.component = '분유'
        message.slot = 'slot_1'

        # PostgreSQL 재고 변경 없이 MongoDB 반복 저장만 확인
        message.status = 'FAILED'
        message.attempt_count = 1
        message.attempts_json = json.dumps([
            {
                'attempt_no': 1,
                'status': 'FAILED',
            },
        ])

        message.error_code = 'MOCK_TEST'
        message.detail = 'Repeated database test'
        message.started_at = self.get_clock().now().to_msg()
        message.ended_at = self.get_clock().now().to_msg()

        self.component_result_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
