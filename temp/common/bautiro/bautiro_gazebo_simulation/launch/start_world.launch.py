# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def _find_world_file(context, *args, **kwargs):
    """Return the full path to the world filename."""
    package_share_dir = get_package_share_path("bautiro_gazebo_simulation")
    world = context.launch_configurations["world"]
    if not world.endswith(".sdf"):
        world = world + ".sdf"
    filename = os.path.join(package_share_dir, "worlds", world)
    if not os.path.isfile(filename):
        raise ValueError(f"File not found: {filename}")
    return [SetLaunchConfiguration("world_filename", filename)]


def generate_launch_description():
    env = {
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ":".join(
            [
                os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
                os.environ.get("LD_LIBRARY_PATH", default=""),
            ]
        )
    }

    actions = []
    # Declare input arguments
    actions.append(
        DeclareLaunchArgument(
            "world",
            default_value="lab_131_pillars.sdf",
            description="Gazebo world filename",
        )
    )

    actions.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Start Gazebo in headless mode",
        )
    )

    # Set opaque function to parse path to world
    actions.append(OpaqueFunction(function=_find_world_file))

    # Call nested launch files
    actions.extend(
        [
            DeclareLaunchArgument(
                "ign_version",
                default_value="6",
                description="Ignition Gazebo's major version",
            ),
            ExecuteProcess(
                cmd=[
                    "ign gazebo",
                    ["-r -v 4 ", LaunchConfiguration("world_filename")],
                    "--force-version",
                    LaunchConfiguration("ign_version"),
                ],
                output="log",
                additional_env=env,
                shell=True,
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
            ExecuteProcess(
                cmd=[
                    "ign gazebo",
                    ["-s -r -v 4 ", LaunchConfiguration("world_filename")],
                    "--force-version",
                    LaunchConfiguration("ign_version"),
                ],
                output="log",
                additional_env=env,
                shell=True,
                condition=IfCondition(LaunchConfiguration("headless")),
            ),
        ]
    )

    return LaunchDescription(actions)
