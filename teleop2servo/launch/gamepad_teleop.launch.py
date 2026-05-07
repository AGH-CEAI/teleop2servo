from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("teleop2servo"), "config", "teleop2servo.yaml"
    )
    container = ComposableNodeContainer(
        name="teleop2servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="teleop2servo",
                plugin="teleop2servo::GamepadTeleopNode",
                name="gamepad_teleop_node",
                parameters=[config_file],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
    )

    return LaunchDescription([container])
