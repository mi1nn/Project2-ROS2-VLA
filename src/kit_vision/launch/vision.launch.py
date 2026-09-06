from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='kit_vision', executable='object_detection', name='object_detection_node'),
        Node(package='kit_vision', executable='debug_view', name='debug_view_node'),
    ])
