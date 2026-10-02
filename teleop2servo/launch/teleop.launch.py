import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("teleop2servo"), "config")

    teleop_device_arg = DeclareLaunchArgument(
        "teleop_device",
        default_value="keyboard",
        choices=["keyboard", "gamepad"],
        description="Teleop input device.",
    )
    use_gamepad = LaunchConfigurationEquals("teleop_device", "gamepad")
    use_keyboard = LaunchConfigurationEquals("teleop_device", "keyboard")

    gamepad_container = ComposableNodeContainer(
        name="teleop2servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=use_gamepad,
        composable_node_descriptions=[
            ComposableNode(package="joy", plugin="joy::Joy", name="joy_node"),
            ComposableNode(
                package="teleop2servo",
                plugin="teleop2servo::TeleopGamepadNode",
                name="gamepad_teleop_node",
                parameters=[os.path.join(config_dir, "teleop_gamepad.yaml")],
            ),
        ],
    )

    keyboard_node = Node(
        package="teleop2servo",
        executable="teleop_keyboard_node",
        name="keyboard_teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(config_dir, "teleop_keyboard.yaml")],
        condition=use_keyboard,
    )

    return LaunchDescription([teleop_device_arg, gamepad_container, keyboard_node])
