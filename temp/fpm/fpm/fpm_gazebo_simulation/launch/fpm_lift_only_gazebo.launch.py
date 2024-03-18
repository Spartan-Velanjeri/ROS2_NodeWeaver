# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # ---------------Paths---------------------------
    # FPM
    urdf_package_path = get_package_share_path('fpm_description')
    default_model_path = urdf_package_path / 'urdf/fpm.urdf.xacro'

    # Rviz configuration
    default_rviz_conf_path = urdf_package_path / 'rviz/urdf.rviz'

    # ----------------Arguments-------------------------
    largs = []
    largs.append(
        DeclareLaunchArgument(
            name='model',
            default_value=str(default_model_path),
            description='Absolute path to robot urdf file'
        )
    )

    largs.append(
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=str(default_rviz_conf_path),
            description='Absolute path to rviz config file'
        )
    )

    largs.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        )
    )

    # Get URDF via xacro
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'lifting_unit',
                   '-allow_renaming', 'true'],
    )
    # State publisher
    joint_state_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('fpm_control'),
             'launch', 'joint_state_broadcaster.launch.py')]))

    # lift controller
    lift_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('fpm_control'),
             'launch', 'lift_controller.launch.py')]))

    return LaunchDescription(
        largs + [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(
                        get_package_share_directory('ros_ign_gazebo'),
                        'launch', 'ign_gazebo.launch.py')]),
                launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])]),
            ignition_spawn_entity,
            joint_state_broadcaster_launch,
            lift_controller_launch,
            robot_state_publisher_node,
        ]
    )
