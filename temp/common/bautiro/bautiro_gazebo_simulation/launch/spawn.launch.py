# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actions = []
    # Declare input arguments
    actions.append(
        DeclareLaunchArgument(
            'robot_name',
            description='Name of the robot to select configuration')
    )
    actions.append(
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Position to spawn (X coordinate)')
    )
    actions.append(
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Position to spawn (Y coordinate)')
    )
    actions.append(
        DeclareLaunchArgument(
            'z',
            default_value='0.4',
            description='Position to spawn (Z coordinate)')
    )
    actions.append(
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Orientation to spawn (roll)')
    )
    actions.append(
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Position to spawn (pitch)')
    )
    actions.append(
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Position to spawn (yaw)'))
    actions.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock')
    )
    actions.append(Node(
        package='ros_ign_gazebo',
        executable='create',
        output="log",
        arguments=['-topic', 'robot_description',
                   '-name', 'bautiro',
                   '-allow_renaming', 'true',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-R', LaunchConfiguration('roll'),
                   '-P', LaunchConfiguration('pitch'),
                   '-Y', LaunchConfiguration('yaw')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    )
    return LaunchDescription(actions)
