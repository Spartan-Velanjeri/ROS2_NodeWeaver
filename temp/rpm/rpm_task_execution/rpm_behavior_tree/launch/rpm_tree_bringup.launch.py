#  Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LA_SIM_TIME = "use_sim_time"
LC_SIM_TIME = LaunchConfiguration(LA_SIM_TIME)

def generate_launch_description():
    # Get the launch directory
    # bringup_dir = get_package_share_directory('rpm_behavior_tree')
    # namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    # params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['rpm_bt']
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            LA_SIM_TIME, default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        Node(
            package='rpm_behavior_tree',
            executable='rpm_bt_navigators',
            output='log',
            parameters=[{LA_SIM_TIME: LC_SIM_TIME}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation2',
            output='log',
            parameters=[{LA_SIM_TIME: LC_SIM_TIME},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
