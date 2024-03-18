# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Launch file only for lift driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file for ctrl_x_driver.

    Returns:
        LaunchDescription: Launch description.
    """
    namespace = LaunchConfiguration(variable_name="namespace")

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="Top level namespace.",
    )

    lift_driver = Node(
        package="fpm_plc_driver",
        executable="lift_driver",
        namespace=namespace,
    )

    return LaunchDescription([namespace_arg, lift_driver])
