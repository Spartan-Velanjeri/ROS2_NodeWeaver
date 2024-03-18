#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# roe2rng (elisa.rothacker@de.bosch.com)
# This launch file launches the Xsens driver
# make sure all included packages here are listed as <exec_depend> in CMakeLists.txt
# The syntax is imported from the respective launch files of the seperate packages
# TODO: unify the parameter definition into one param file for readability

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------Launch Arguments-------------------------
    # mode = LaunchConfiguration('mode')
    node_name = LaunchConfiguration('name')
    driver_package_name = LaunchConfiguration('driver_package')
    executable = LaunchConfiguration('executable')
    namespace = LaunchConfiguration('namespace')
    parameter_file = LaunchConfiguration('parameters')
    params_declare = DeclareLaunchArgument(
        name='parameters',
        default_value=os.path.join(
            get_package_share_directory('rpm_sensors'), 'config', 'RPM', 'xsens_mti_node.yaml'),
        description='Path to the ROS2 parameters file to use for XSens Driver.'
    )
    mode_declare = DeclareLaunchArgument(
        name="mode",
        default_value="gazebo_sim",
        description="RPM_LOCALIZATION mode: simulation or real hardware (real, gazebo_sim)",
    )
    node_name_declare = DeclareLaunchArgument(
        name="name",
        default_value="",
        description="node_name",
    )
    package_name_declare = DeclareLaunchArgument(
        name="driver_package",
        default_value="",
        description="driver_package",
    )
    executable_declare = DeclareLaunchArgument(
        name="executable",
        default_value="",
        description="executable",
    )
    namespace_declare = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )

    imu_node_name = node_name
    imu_node = Node(
        package=driver_package_name,
        executable=executable,
        name=imu_node_name,
        namespace=namespace,
        output='screen',
        parameters=[parameter_file],
        arguments=[]
    )

    ld.add_action(imu_node)
    ld.add_action(params_declare)
    ld.add_action(mode_declare)
    ld.add_action(node_name_declare)
    ld.add_action(package_name_declare)
    ld.add_action(executable_declare)
    ld.add_action(namespace_declare)

    return ld
