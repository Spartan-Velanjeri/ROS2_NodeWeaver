# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    actions = []
    actions.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value='view_bautiro.rviz',
            description='YAML file with the controllers configuration.',
        )
    )

    actions.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock')
    )

    actions.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='FUS1',
            description='Name of the robot to select configuration '
                        'and calibration files.')
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('bautiro_description'),
         'rviz', LaunchConfiguration('rviz_config')]
    )

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bautiro_description'),
                'launch', 'robot.launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_name': LaunchConfiguration('robot_name')}.items()))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    ))

    # Joint state publisher
    actions.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        arguments=[PathJoinSubstitution([
            FindPackageShare('bautiro_description'),
            'robots', LaunchConfiguration('robot_name'), 'bautiro.urdf']
        )])
    )

    return LaunchDescription(actions)
