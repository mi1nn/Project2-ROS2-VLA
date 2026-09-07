from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug = LaunchConfiguration("debug")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description=(
                    "Open the lightweight debug viewer. "
                    "It does not run a second YOLO model."
                ),
            ),

            Node(
                package="kit_vision",
                executable="object_detection",
                name="object_detection_node",
                output="screen",
            ),

            Node(
                package="kit_vision",
                executable="debug_view",
                name="debug_view_node",
                output="screen",
                condition=IfCondition(debug),
            ),
        ]
    )
