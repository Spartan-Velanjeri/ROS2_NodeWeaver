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
    # General arguments
    controller_package = LaunchConfiguration("controller_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_package_rpm = LaunchConfiguration("description_package_rpm")
    description_package_fpm = LaunchConfiguration("description_package_fpm")
    description_file = LaunchConfiguration("description_file")
    rpm_name = LaunchConfiguration("rpm_name")
    fpm_name = LaunchConfiguration("fpm_name")
    robot_name = LaunchConfiguration("robot_name")
    # prefix = LaunchConfiguration("prefix")

    # ---------------Paths---------------------------

    default_controllers = PathJoinSubstitution(
        [FindPackageShare(controller_package), "config", controllers_file]
    )

    calibration_file_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", robot_name, "calibration_bautiro.yaml"]
    )
    calibration_file_path_rpm = PathJoinSubstitution(
        [FindPackageShare(description_package_rpm), "config", rpm_name, "calibration_rpm.yaml"]
    )
    calibration_file_path_fpm = PathJoinSubstitution(
        [FindPackageShare(description_package_fpm), "config", fpm_name, "calibration_fpm.yaml"]
    )

    # bus configuration
    # bus_config_package = LaunchConfiguration("bus_config_package")
    # bus_config_directory = LaunchConfiguration("bus_config_directory")
    # bus_config_file = LaunchConfiguration("bus_config_file")
    # bus configuration file full path
    # bus_config = PathJoinSubstitution(
    #     [FindPackageShare(bus_config_package), bus_config_directory, bus_config_file]
    # )

    # master configuration
    # master_config_package = LaunchConfiguration("master_config_package")
    # master_config_directory = LaunchConfiguration("master_config_directory")
    # master_config_file = LaunchConfiguration("master_config_file")
    # master configuration file full path
    # master_config = PathJoinSubstitution(
    #     [FindPackageShare(master_config_package), master_config_directory, master_config_file]
    # )

    # can interface name
    # can_interface_name = LaunchConfiguration("can_interface_name")

    # robot description stuff
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", robot_name, description_file]
            ),
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur16e",
            " ",
            "prefix:=",
            "hu_",
            " ",
            "ur_parent:=",
            "handling_unit_base",
            " ",
            "sim_ignition:=false",
            " ",
            "simulation_controllers:=",
            default_controllers,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "calibration_file:=",
            calibration_file_path,
            " ",
            "calibration_file_rpm:=",
            calibration_file_path_rpm,
            " ",
            "calibration_file_fpm:=",
            calibration_file_path_fpm,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
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

    # ------------------Nodes-------------------------
    # Robot state publisher
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )

    # ros2 control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name='rpm_controller_manager',
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

    nodes_to_start = [
        control_node,
        # robot_state_publisher_node,
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
            "description_package",
            default_value="bautiro_description",
            description="Package where urdf file is stored.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_rpm",
            default_value="rpm_description",
            description="Description package with URDF/XACRO files of rpm. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package_fpm",
            default_value="fpm_description",
            description="Description package with URDF/XACRO files of fpm. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="bautiro.urdf.xacro",
            description="Name of the urdf file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            description="robot name",
            default_value="FUS1",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rpm_name",
            default_value="RPM1",
            description="name of the rpm entity",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fpm_name",
            default_value="FPM1",
            description="name of the fpm entity",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_package",
            default_value="ur_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("prefix", description="Prefix.", default_value="")
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
