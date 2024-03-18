# Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
# All rights reserved.
#
# Made for Robert Bosch GmbH
#
# Proprietary License
#
# Unauthorized copying of this file, via any medium is strictly prohibited.
# The file is considered confidential.

#
# Author: Dr. Denis (denis.stogl@stoglrobotics.de)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    # master configuration
    master_config_package = LaunchConfiguration("master_config_package")
    master_config_directory = LaunchConfiguration("master_config_directory")
    # master_config_file = LaunchConfiguration("master_config_file")

    # hardcoded slave configuration form test package
    slave_config = PathJoinSubstitution(
        [FindPackageShare(master_config_package), master_config_directory, "fake_inverter.eds"]
    )
    slave_launch = PathJoinSubstitution(
        [FindPackageShare("rpm_powertrain_driver"), "launch", "fake_inverter.launch.py"]
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "0x26",
            "node_name": "left_wheel_joint_fake_slave",
            "slave_config": slave_config,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "0x27",
            "node_name": "right_wheel_joint_fake_slave",
            "slave_config": slave_config,
        }.items(),
    )

    nodes_to_start = [
        slave_node_1,
        slave_node_2,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_package",
            default_value="rpm_actuators",
            description="Path to master configuration file (*.dcf)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_directory",
            default_value="config/mobile_base",
            description="Path to master configuration file (*.dcf)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_file",
            default_value="master.dcf",
            description="Path to master configuration file (*.dcf)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="Interface name for can",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
