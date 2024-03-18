# Copyright 2020 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
#
# 2023.07.13: Michael Erz
# This is a copy of 'CRSLAM/cr_slam/slam_launch/slam_launch/launch_helper.py'
# with modified SLAM_NODE_GENERATOR

# from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# from launch.substitutions import LaunchConfiguration

import os
import yaml

# import xacro


def slam_node_generator(
    context: LaunchContext, slam_launch_args: dict, config_files: list
) -> Node:
    """
    This function uses the config_files and launch file arguments to
    configure the SLAM node and finally returns a Node object that can be
    used by LaunchDescription to start the SLAM pipeline.
    :param context: the context object necessary for accessing argument values
    :param config_files: list of yaml filepaths containing SLAM configs
    """

    slam_config = {}

    # Set the cwd of the node to $ROS_HOME, like in ROS1
    ros_home_directory = os.environ.get(
        "ROS_HOME", default=os.path.join(os.environ.get("HOME"), ".ros")
    )

    # Load the "normal" config files first that are hardcoded in the launch file
    for yaml_filename in config_files:
        config_file = yaml_filename

        print("[SLAM] Loading config from " + config_file)
        with open(config_file, "r") as f:
            single_config = yaml.safe_load(f.read())
            slam_config.update(single_config)

    # Set arguments
    arguments = {}
    for arg_name, arg_value in context.launch_configurations.items():
        # If a launch file arg is set to "", its corresponding ros param should not be set
        # Also ignore parameters that are not present
        if (
            arg_value != ""
            and arg_name in slam_launch_args
            and slam_launch_args[arg_name]["param_name"] in slam_config
        ):
            slam_config[slam_launch_args[arg_name]["param_name"]] = arg_value
            arguments[arg_name] = arg_value

    # Load the additional config
    if "additional_config" in arguments:
        additional_config = arguments.pop("additional_config")
        print("[SLAM] Loading additional_config from: " + additional_config)
        with open(additional_config, "r") as f:
            single_config = yaml.safe_load(f.read())
            slam_config.update(single_config)

    if "prefix" in arguments:
        prefix = arguments.pop("prefix")
    else:
        prefix = ""

    return Node(
        cwd=ros_home_directory,
        package="slam_control",
        executable="new_slam_node",
        prefix=prefix,
        name="slam_node",
        output="screen",
        # later parameters overwrite earlier ones
        parameters=[slam_config, arguments],
    )


def slam_launch(
    slam_launch_args: dict, config_files: list, rviz_files: list = []
) -> LaunchDescription:
    """
    Create a LaunchDescription instance to start a fully configured SLAM node

    :param slam_launch_args: Definitions of the launch file arguments the SLAM
                             launch file should have. Dict keys are arg_names,
                             contains another dict with default value, description
                             and the node's respective parameter name.
    :param config_files: yaml files containing SLAM configs.
                         Files loaded from slam_control/config
    :param rviz_files: compound of xacro and rviz filepaths.
                       rviz_files[0] = path-to-xacrofile (unused),
                       rviz_files[1] = path-to-rvizconfig
    """
    ld = LaunchDescription()

    for arg_name, arg_defs in slam_launch_args.items():
        ld.add_action(
            DeclareLaunchArgument(
                name=arg_name,
                default_value=arg_defs["default"],
                description=arg_defs["description"],
            )
        )

    def load_params_and_start_slam(context):
        return [slam_node_generator(context, slam_launch_args, config_files)]

    ld.add_action(OpaqueFunction(function=load_params_and_start_slam))

    # To uncomment when usecases are ready
    # if len(rviz_files):
    #     [xacro_action, rviz_action] = rviz_node_generator(rviz_files)
    #     ld.add_action(rviz_action)

    return ld
