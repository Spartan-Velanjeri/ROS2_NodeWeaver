# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    actions = []
    # Declare input arguments
    actions.append(
        DeclareLaunchArgument(
            "initial_ur_joint_controller",
            default_value="forward_position_controller",
            description="Robot controller to start.",
        )
    )

    # UR controller
    spawn_controller_action = ExecuteProcess(
        cmd=[
            ExecutableInPackage(executable="spawner", package="controller_manager"),
            LaunchConfiguration("initial_ur_joint_controller"),
            "--controller-manager",
            "/controller_manager",
        ],
    )

    actions.append(spawn_controller_action)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_controller_action,
                on_exit=[
                    LogInfo(msg="Sending arm to rest position"),
                    ExecuteProcess(
                        cmd=[
                            ExecutableInPackage(
                                executable="arm_to_rest.py",
                                package="bautiro_gazebo_simulation",
                            )
                        ],
                    ),
                ],
            )
        )
    )

    # Lift controller
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("fpm_control"),
                        "launch",
                        "lift_controller.launch.py",
                    )
                ]
            )
        )
    )

    # RPM controller
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("rpm_control"),
                        "launch",
                        "rpm_controller.launch.py",
                    )
                ]
            )
        )
    )

    return LaunchDescription(actions)
