# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    la = DeclareLaunchArgument(
        'my_keyword',
        default_value='info')

    lc = LaunchConfiguration('my_keyword')

    return LaunchDescription([
        la,
        Node(
            package='ccu_hcu_abstraction',
            executable='main',
            arguments=['--ros-args', '--log-level', lc],
        )
    ])
