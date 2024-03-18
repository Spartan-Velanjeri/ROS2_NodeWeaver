#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

import os
from ament_index_python.packages import get_package_share_directory

from controller_manager.launch_utils import generate_load_controller_launch_description
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # use odom
    enable_odom_tf = LaunchConfiguration("enable_odom_tf")
    config_file = os.path.join(
        get_package_share_directory('rpm_control'),
        'config', 'rpm_control_no_odom.yaml')

    if enable_odom_tf:
        config_file = os.path.join(
            get_package_share_directory('rpm_control'),
            'config', 'rpm_control_sim.yaml')

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_base_controller", "--controller-manager", "/controller_manager"],
    # )

    x = generate_load_controller_launch_description(
        controller_name='rpm_velocity_controller',
        controller_type='diff_drive_controller/DiffDriveController',
        controller_params_file=config_file)
    x.add_entity(
        DeclareLaunchArgument(
            "enable_odom_tf",
            default_value="true",
            description='Enable/Disable odom -> base_link transformation.'
        )
    )

    return x
