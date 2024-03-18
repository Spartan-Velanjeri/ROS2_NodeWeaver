# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os
import sys
from enum import Enum

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LocalSubstitution
from launch_ros.actions import PushRosNamespace
from rclpy.logging import get_logger

LOGGER = get_logger(__name__)


class FPM_HW_Version(Enum):
    """Hardware version of the FPM."""

    FUS1 = "FUS1"
    FUS2 = "FUS2"


FPM_COMPONENTS = {
    "lift": {
        "package": "fpm_ctrl_x_driver",
        "hw_version": [FPM_HW_Version.FUS1],
        "launch": "lift.launch.py",
        "namespace": "",
    },
    "vacuum": {
        "package": "fpm_ctrl_x_driver",
        "hw_version": [FPM_HW_Version.FUS1],
        "launch": "vacuum.launch.py",
        "namespace": "",
    },
    "fpm_coordinator": {
        "package": "fpm_coordinator",
        "hw_version": [FPM_HW_Version.FUS1],
        "launch": "fpm_coordinator.launch.py",
        "namespace": "",
    },
    # "inclinometer": {
    #     "package": "fpm_inclinometer_driver",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "main.launch.py",
    #     "namespace": "fpm/sensors/inclinometer",
    # },
    # "fpm_ptu_driver": {
    #     "package": "fpm_ptu_driver",
    #     "hw_version": [],
    #     "launch": "fpm_coordinator.launch.py",
    #     "namespace": "fpm/actuators/ptu"
    # },
    # "imu": {
    #     "package": "bluespace_ai_xsens_mti_driver",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "xsens_mti_node.launch.py",
    #     "namespace": "fpm/sensors/imu",
    # },
    "fpm_behavior_tree": {
        "package": "fpm_behavior_tree",
        "hw_version": [FPM_HW_Version.FUS1],
        "launch": "skill_server.launch.py",
        "namespace": "",
    },
    # "ur": {
    #     "package": "ur_bringup",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "ur_control.launch.py",
    #     "parameters": {
    #         "ur_type": "ur16e",
    #         "robot_ip": "192.168.2.3",
    #         "launch_rviz": "false",
    #         "prefix": "hu_",
    #         "initial_joint_controller": "joint_trajectory_controller",
    #     },
    #     "namespace": "fpm",
    # },
    # "fpm_moveit": {
    #     "package": "fpm_moveit",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "ur_moveit.launch.py",
    #     "parameters": {"launch_rviz": "false"},
    #     "namespace": "fpm",
    # },
    # "hu_motion_manager": {
    #     "package": "handling_unit_motion_manager",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "main.launch.py",
    #     "namespace": "fpm",
    # },
    # "hu_behavior_tree": {
    #     "package": "hu_behavior_tree",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "hu_tree_bringup.launch.py",
    #     "namespace": "fpm",
    # },
    # "leica_16": {
    #     "package": "ts16",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "rs232_driver.py",
    #     "namespace": "fpm/sensors/total_station",
    #     "parameters": {"serial_port": "/dev/le16"},
    # },
    # "lu_fein_localization": {
    #     "package": "fpm_coordinator",
    #     "hw_version": [FPM_HW_Version.FUS1],
    #     "launch": "fpm_coordinator.launch.py",
    # },
}


def generate_launch_description():
    """Return FPM launch description."""
    hw_version = None

    for arg in sys.argv:
        if "hw_version" in arg:
            hw_version = FPM_HW_Version(arg.split(":=")[1])
            break

    LOGGER.info(f'Creating launch configuration for HW version "{hw_version}".')
    launch_descriptions = []

    # launch arguments
    hardware_version = DeclareLaunchArgument(
        name="hw_version",
        description="Hardware version of the FPM system.",
        choices=[e.value for e in FPM_HW_Version],
    )
    stdout_log = SetEnvironmentVariable(
        name="RCUTILS_LOGGING_BUFFERED_STREAM", value="1"
    )

    launch_descriptions.append(hardware_version)
    launch_descriptions.append(stdout_log)

    # create launch descriptions for ROS nodes
    for component, config in FPM_COMPONENTS.items():
        launch_script = config["launch"]
        share_directory = config["package"]
        namespace = config["namespace"]
        parameters = config.get("parameters", {})

        if hw_version not in config["hw_version"]:
            LOGGER.info(f"Excluded {component} from launch definition.")
            continue

        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory(share_directory),
                        "launch",
                    ),
                    f"/{launch_script}",
                ]
            ),
            launch_arguments=parameters.items(),
        )

        launch_description_with_namespace = GroupAction(
            actions=[
                PushRosNamespace(namespace),
                launch_description,
            ]
        )

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

        LOGGER.info(f"Adding {component} to launch definition.")
        launch_descriptions.append(launch_description_with_namespace)
        launch_descriptions.append(shutdown_handler)

    return LaunchDescription(launch_descriptions)
