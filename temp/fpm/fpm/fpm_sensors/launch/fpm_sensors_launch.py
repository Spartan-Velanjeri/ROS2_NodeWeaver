# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch.actions import (
    DeclareLaunchArgument,
    LogInfo, OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers.on_shutdown import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, \
    PythonExpression, LocalSubstitution
from rclpy.logging import get_logger

LOGGER = get_logger(__name__)

# single use e.g. via ros2 launch fpm_sensors fpm_sensors_launch.py mode:=real hw_sample:=FPM1
# config_file_with_path:=/home/roe2rng/test_ws_ros2/install/fpm_sensors/share/fpm_sensors/config/FPM1/fpm_config.yaml


# Setup Launch configuration
def launch_setup(context, *args, **kwargs):
    # Get launch arguments

    # This argument is name of the hardware sample
    # for bautiro: FUS1, FUS2,...
    # for RPM: RPM1, RPM2, ...
    # for FPM: FPM1, FPM2, ...
    hw_sample = LaunchConfiguration("hw_sample").perform(context)

    # This is running mode on real robot or simulation
    # possible value: real, gazebo, ...
    mode = LaunchConfiguration("mode").perform(context)

    # Namespace to launch components
    # namespace = LaunchConfiguration("namespace").perform(context)

    # config file with path
    config_file_with_path = LaunchConfiguration("config_file_with_path").perform(context)

    # Create launch description for launch file
    launch_descriptions = []
    LOGGER.info(f"used config file: {config_file_with_path} ")
    # read config file
    with open(config_file_with_path, 'r') as f:
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
            get_package_share_directory(package_name), "launch", launch_script)
        LOGGER.info(f"Including Sensor Launch File: {launch_file_path} .")

        params_file_path = os.path.join(
            get_package_share_directory(package_name), "config", hw_sample, params)

        # Create launch description for launch component
        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),
                "launch", launch_script
            )]),
            launch_arguments={
                'mode': mode,
                'config_file_with_path': config_file_with_path,
                'name': node_name,
                'namespace': ns,
                'driver_package': driver_package_name,
                'executable': executable,
                'parameters': [params_file_path],
            }.items()
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
            default_value="FPM1",
            description='Name of the Hardware Sample',
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="gazebo_sim",
            description='FPM_LOCALIZATION mode: simulation or real hardware (gazebo_sim, real)',
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "config_file_with_path",
            default_value=os.path.join(
                get_package_share_directory('fpm_sensors'), 'config',
                'FPM1', 'fpm_config.yaml'),
            description='Name of config file',
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description='Namespace',
        )
    )

    launch_arguments.append(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'real'),
                msg=PythonExpression(
                    expression=["'Running in ' + '",
                                LaunchConfiguration('mode'),
                                "'",
                                " + ' mode ...'"])
                )
    )

    launch_arguments.append(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'gazebo_sim'),
                msg=PythonExpression(
                    expression=["'Running in ' + '",
                                LaunchConfiguration('mode'),
                                "'",
                                " + ' mode ...'"])
                )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
