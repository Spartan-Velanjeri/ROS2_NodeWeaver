"""Launch file only for lift driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file for fpm_coordinator.

    Returns:
        LaunchDescription: Launch description.
    """
    namespace = LaunchConfiguration(variable_name="namespace")

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="Top level namespace.",
    )

    coordinator = Node(
        package="fpm_coordinator",
        executable="coordinator",
        namespace=namespace,
    )

    return LaunchDescription([namespace_arg, coordinator])
