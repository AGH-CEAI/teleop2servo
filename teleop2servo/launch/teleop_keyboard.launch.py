from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("teleop2servo")
    default_params = os.path.join(pkg, "config", "teleop_keyboard.yaml")

    params_file = LaunchConfiguration("params_file")
    node_name = LaunchConfiguration("node_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file with teleop keyboard parameters"
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="teleop_keyboard",
            description="Node name"
        ),
        Node(
            package="teleop2servo",
            executable="teleop_keyboard_node",
            name=node_name,
            output="screen",
            parameters=[params_file],
        ),
    ])