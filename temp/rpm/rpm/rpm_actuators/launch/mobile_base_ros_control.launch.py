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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")

    # bus configuration
    bus_config_package = LaunchConfiguration("bus_config_package")
    bus_config_directory = LaunchConfiguration("bus_config_directory")
    bus_config_file = LaunchConfiguration("bus_config_file")
    # bus configuration file full path
    bus_config = PathJoinSubstitution(
        [FindPackageShare(bus_config_package), bus_config_directory, bus_config_file]
    )

    # master configuration
    master_config_package = LaunchConfiguration("master_config_package")
    master_config_directory = LaunchConfiguration("master_config_directory")
    master_config_file = LaunchConfiguration("master_config_file")
    # master configuration file full path
    master_config = PathJoinSubstitution(
        [FindPackageShare(master_config_package), master_config_directory, master_config_file]
    )

    # can interface name
    can_interface_name = LaunchConfiguration("can_interface_name")

    # robot description stuff
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "RPM1", description_file]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "is_sim:=false",
            " ",
            "bus_config:=",
            bus_config,
            " ",
            "master_config:=",
            master_config,
            " ",
            "can_interface_name:=",
            can_interface_name,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ros2 control configuration
    ros2_control_config_package = LaunchConfiguration("ros2_control_config_package")
    ros2_control_config_directory = LaunchConfiguration("ros2_control_config_directory")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    # ros2 control configuration file full path
    ros2_control_config = PathJoinSubstitution(
        [
            FindPackageShare(ros2_control_config_package),
            ros2_control_config_directory,
            ros2_control_config_file,
        ]
    )

    # nodes to start are listed below
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_config],
        output="screen",
    )

    # load one controller just to make sure it can connect to controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_base_controller", "-c", "/controller_manager"],
    )

    left_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_wheel_controller",
            "-t", "canopen_ros2_controllers/CanopenProxyController",
            "-c", "/controller_manager"],
    )

    right_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_wheel_controller",
            "-t", "canopen_ros2_controllers/CanopenProxyController",
            "-c", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        diff_base_controller_spawner,
        left_wheel_controller,
        right_wheel_controller,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "name", description="robot name", default_value="fake_rpm_test_system"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", description="Prefix.", default_value="")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where urdf file is stored.",
            default_value="rpm_description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            description="Name of the urdf file.",
            # default_value="rpm_chassis.urdf.xacro",
            default_value="rpm.urdf.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_package",
            default_value="rpm_actuators",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_directory",
            default_value="config_ros2_control",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value="mobile_base_ros2_control.yaml",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_package",
            default_value="rpm_actuators",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_directory",
            default_value="config/mobile_base_old",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_file",
            default_value="bus.yml",
            description="Path to bus configuration.",
        )
    )
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
            default_value="config/mobile_base_old",
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
            default_value="can0",
            description="Interface name for can",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
