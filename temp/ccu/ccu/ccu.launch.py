# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration as Lc
from launch_ros.actions import Node


def generate_launch_description():

    lifecycle_nodes = ['ccu_bt']

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart',
                              default_value='true',
                              description='Automatically startup the nav2 stack'),
        Node(package='ccu_data_services',
             executable='data_service',
             namespace=Lc("namespace"),
             output='screen'),
        Node(package='ccu_hcu_abstraction',
             executable='halc',
             namespace=Lc("namespace"),
             output='screen'),
        Node(package='ccu_behavior_tree',
             executable='ccu_bt_navigators',
             name='ccu_bt',
             output='screen'),
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             namespace=Lc("namespace"),
             name='lifecycle_manager_navigation2',
             output='screen',
             parameters=[{'use_sim_time': Lc('use_sim_time')},
                         {'autostart': Lc('autostart')},
                         {'node_names': lifecycle_nodes}]),
    ])
