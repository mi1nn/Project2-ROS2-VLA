'''
controller 테스트용 데모 파일
개발 완료 후 삭제 예정
'''
import json

import rclpy
from rclpy.node import Node

from kit_interfaces.srv import (
    GetCommand,
    GetComponentPose,
    InspectKit,
)


class DemoServices(Node):
    def __init__(self):
        super().__init__("controller_demo_services")

        self.create_service(
            GetCommand, "/get_command", self.get_command
        )
        self.create_service(
            GetComponentPose, "/get_component_pose", self.get_pose
        )
        self.create_service(
            InspectKit, "/inspect_kit", self.inspect_kit
        )

    def get_command(self, request, response):
        self.get_logger().info(f"명령 요청: {request.task_id}")

        response.success = True
        response.command_json = json.dumps({
            "kit_type": "demo",
            "items": [{"name": "컵라면", "qty": 1}],
        }, ensure_ascii=False)
        response.error_code = ""
        return response

    def get_pose(self, request, response):
        self.get_logger().info(f"좌표 요청: {request.component}")

        response.success = True
        response.target_pose = [0.0] * 6
        response.error_code = ""
        return response

    def inspect_kit(self, request, response):
        self.get_logger().info("검사 요청")

        response.ok = True
        response.actual_counts = list(request.expected_counts)
        response.missing = []
        response.unexpected = []
        response.detection_age = 0.1
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DemoServices()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
