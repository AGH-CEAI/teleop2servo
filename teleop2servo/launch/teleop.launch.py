from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    teleop_device = LaunchConfiguration("teleop_device")

    gamepad_config_file = os.path.join(
        get_package_share_directory("teleop2servo"),
        "config",
        "teleop_gamepad.yaml",
    )
    keyboard_config_file = os.path.join(
        get_package_share_directory("teleop2servo"),
        "config",
        "teleop_keyboard.yaml",
    )

    gamepad_container = ComposableNodeContainer(
        name="teleop2servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # MultiThreadedExecutor
        output="screen",
        condition=IfCondition(PythonExpression(["'", teleop_device, "' == 'gamepad'"])),
        composable_node_descriptions=[
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                # TODO(PR#4) Construct whole launch file logic for Keyboard/Gamepad selection
                # condition=IfCondition(cfg["launch_rviz"]),
            ),
            ComposableNode(
                package="teleop2servo",
                plugin="teleop2servo::TeleopGamepadNode",
                name="gamepad_teleop_node",
                parameters=[gamepad_config_file],
            ),
        ],
    )

    keyboard_node = Node(
        package="teleop2servo",
        executable="teleop_keyboard_node",
        name="teleop_keyboard_node",
        output="screen",
        emulate_tty=True,
        parameters=[keyboard_config_file],
        condition=IfCondition(
            PythonExpression(["'", teleop_device, "' == 'keyboard'"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop_device",
                default_value="keyboard",
                description="Teleop input device: gamepad, keynoard or none.",
            ),
            gamepad_container,
            keyboard_node,
        ]
    )
