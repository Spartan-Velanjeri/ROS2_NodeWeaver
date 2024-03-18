# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart',
            description='Automatically startup the nav2 stack'),

        Node(
            package='fpm_ceiling_distance',
            executable='ceiling_distance_node',
            name='ceiling_distance',
            output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_ceiling_distance',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['ceiling_distance']}]),

    ])
