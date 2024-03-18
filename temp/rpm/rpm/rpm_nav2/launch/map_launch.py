#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

LA_SIM_TIME = "use_sim_time"
LC_SIM_TIME = LaunchConfiguration(LA_SIM_TIME)


def generate_launch_description():

    ld = LaunchDescription()

    # Map server
    map_server_config_path = os.path.join(
        get_package_share_directory('rpm_nav2'),
        'maps',
        'le131_Revit_nav.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='log',
        parameters=[{'yaml_filename': map_server_config_path,
                     LA_SIM_TIME: LC_SIM_TIME}])

    lifecycle_nodes = ['map_server']
    autostart = True

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='log',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{LA_SIM_TIME: LC_SIM_TIME},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    return ld
