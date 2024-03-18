#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

# bes2rng (stefan.benz@de.bosch.com)
# This launch file launches all drivers for the bautiro localization waegele
# make sure all included packages here are listed as <exec_depend> in CMakeLists.txt
# The syntax is imported from the respective launch files of the seperate packages
# TODO: unify the parameter definition into one param file for readability

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_shutdown import OnShutdown
from launch.substitutions import LocalSubstitution
from rclpy.logging import get_logger
import os

CURRENT_PACKAGE = "rpm_sensors"

LOGGER = get_logger(__name__)

# config_file_with_path = os.path.join(get_package_share_directory('rpm_sensors'),
#                         "config","RPM1",'rpm_config.yaml'
#                     )


# Setup Launch configuration
def launch_setup(context, *args, **kwargs):
    # Get launch arguments

    # This argument is name of the hardware sample
    # FUS1, FUS2,...
    hw_sample = LaunchConfiguration("hw_sample").perform(context)

    # backwards compat for name
    if hw_sample.startswith("RPM"):
        LOGGER.warn("RPM-name is deprecated, use FUSx directly")
        hw_sample = hw_sample.replace("RPM", "FUS")

    # This is running mode on real robot or simulation
    # possible value: real, gazebo, ...
    mode = LaunchConfiguration("mode").perform(context)

    # Namespace to launch components
    namespace = LaunchConfiguration("namespace").perform(context)

    # config file with path
    config_file_with_path = (
        os.path.join(
            get_package_share_directory("rpm_sensors"),
            "config",
            hw_sample,
            "rpm_config.yaml",
        ),
    )

    # Create launch description for launch file
    launch_descriptions = []
    LOGGER.info(f"used config file: {config_file_with_path} ")
    # read config file
    with open(config_file_with_path, "r") as f:
        config_file = yaml.load(f, Loader=yaml.SafeLoader)

    # create launch descriptions
    for component, config in config_file.items():

        # Get launch file and package of launch component
        launch_script = config_file[component]["launch_file"]
        package_name = config_file[component]["package"]
        node_name = config_file[component]["name"]
        ns = config_file[component]["namespace"]
        params = config_file[component]["parameters"]
        driver_package_name = config_file[component]["driver_package"]
        executable = config_file[component]["executable"]

        launch_file_path = os.path.join(
            get_package_share_directory(package_name), "launch", launch_script
        )
        LOGGER.info(f"Including Sensor Launch File: {launch_file_path} .")

        params_file_path = os.path.join(
            get_package_share_directory(package_name), "config", hw_sample, params
        )

        # Create launch description for launch component
        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory(package_name),
                        "launch",
                        launch_script,
                    )
                ]
            ),
            launch_arguments={
                "mode": mode,
                "config_file_with_path": config_file_with_path,
                "name": node_name,
                "namespace": ns,
                "driver_package": driver_package_name,
                "executable": executable,
                "parameters": [params_file_path],
            }.items(),
        )

        # Push launch component to namespace
        launch_description_with_namespace = GroupAction(
            actions=[
                # PushRosNamespace(ns),
                launch_description,
            ]
        )

        # Handler on Shutdown
        shutdown_handler = RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    LogInfo(
                        msg=[
                            "Launch was asked to shutdown: ",
                            LocalSubstitution("event.reason"),
                        ]
                    )
                ]
            )
        )

        # Add launch description

        LOGGER.info(f"Adding {component} to launch definition.")
        launch_descriptions.append(launch_description_with_namespace)
        launch_descriptions.append(shutdown_handler)

    return launch_descriptions


def generate_launch_description():
    # ----------------Launch Arguments-------------------------
    launch_arguments = []
    # General arguments
    launch_arguments.append(
        DeclareLaunchArgument(
            "hw_sample",
            choices=["FUS1", "FUS2", "FUS3"],
            description="Name of the Hardware Sample",
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "mode",
            description="Name of the mode (real, gazebo_sim)",
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace",
        )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
