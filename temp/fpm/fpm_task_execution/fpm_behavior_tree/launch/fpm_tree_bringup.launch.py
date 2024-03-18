# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('fpm_behavior_tree')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
                       'fpm_bt_navigators']


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),


        
        # Node(
        #     package='ccu_data_services',
        #     executable='data_service',
        #     name='data_service',
        #     output='screen'),  
        Node(
            package='fpm_behavior_tree',
            executable='fpm_bt_navigators',
            # name='fpm_bt_navigators',
            output='screen'),
        # Node(
        #     package='fpm_behavior_tree',
        #     executable='sim_fpm_bt_navigators',
        #     # name='fpm_bt_navigators',
        #     output='screen'),
        # Node(
        #     package='ccu_behavior_tree',
        #     executable='rpm_bt_navigators',
        #     name='rpm_bt_navigators',
        #     output='screen'),
            
	#Node(
            #package='nav2_recoveries',
            #executable='recoveries_server',
            #name='recoveries_server',
            #output='screen'),
    #     Node(
    #         package='cluster_components',
    #         executable='load_points_service',
    #    parameters=[os.path.join(get_package_share_directory('fpm_handling_unit_script_switching'),
	#                'config', 'cluster_points.yaml')],
    #         output='screen'),  
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
