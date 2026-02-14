from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="teleop2servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # or "component_container"
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="teleop2servo",
                plugin="teleop2servo::JoyToServoPubAegis",
                name="joy_to_twist_publisher",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
    )

    return LaunchDescription([container])
