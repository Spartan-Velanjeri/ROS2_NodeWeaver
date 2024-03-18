# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    # Gazebo Simulation
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('bautiro_gazebo_simulation'),
                          'launch', 'start_world.launch.py')]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('bautiro_gazebo_simulation'),
                          'launch', 'spawn.launch.py')]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_name': LaunchConfiguration('robot_name')}.items()
    )

    start_static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('bautiro_gazebo_simulation'),
                          'launch', 'start_static_transforms.launch.py')]))

    start_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('bautiro_gazebo_simulation'),
                          'launch', 'start_bridge.launch.py')]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    start_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('bautiro_gazebo_simulation'),
                          'launch', 'start_controllers.launch.py')]))

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('bautiro_gazebo_simulation'),
         'rviz', 'simulation_rviz.rviz']
    )
    # Visualization
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        executable='rviz2',
        name='rviz2',
        output={'stdout': 'log', 'stderr': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration("use_sim_time")
        }]
    )

    launch = [
        start_gazebo,
        spawn_robot,
        start_controllers,
        start_static_transforms,
        start_bridge,
        rviz_node,
    ]

    return launch


def generate_launch_description():

    # ----------------Arguments-------------------------
    actions = []
    actions.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Boolean value to whether or not launch rviz')
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
            description='Name of the robot to select configuration')
    )
    actions.append(
        DeclareLaunchArgument(
            "world",
            default_value="lab_131_pillars.sdf",
            description="Gazebo world filename",
        )
    )
    return LaunchDescription(actions + [OpaqueFunction(function=launch_setup)])
