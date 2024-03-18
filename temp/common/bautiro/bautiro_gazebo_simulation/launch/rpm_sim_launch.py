"""Run RPM in simulation."""

# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch_ros.actions import Node

from bautiro_launch.paths import get_launch_pkg
from bautiro_launch.stdargs import (
    LA_MODE,
    LA_SIM_TIME,
    SIM_ONLY,
    LA_HEADLESS,
    LC_HEADLESS,
    get_std_lvl1_arg_declares,
    get_std_lvl2_arguments,
)


def generate_launch_description():
    return LaunchDescription(
        [
            SetLaunchConfiguration(LA_MODE, "gazebo_sim"),
            SetLaunchConfiguration(LA_SIM_TIME, "True"),
        ] + get_std_lvl1_arg_declares() + [
            # configure sim time based on mode
            Node(
                name="check_sim_time",
                package="bautiro_launch",
                executable="check_sim_time_node",
                arguments=[("__log_level:=error")],
                condition=SIM_ONLY,
            ),
            # now include the simulation
            IncludeLaunchDescription(
                [
                    get_launch_pkg(
                        "bautiro_gazebo_simulation",
                        "bautiro_spawn_gazebo_control.launch.py",
                    ),
                ],
                condition=SIM_ONLY,
                launch_arguments=get_std_lvl2_arguments() + [(LA_HEADLESS, LC_HEADLESS)],
            ),
            # and the RPM
            IncludeLaunchDescription(
                [
                    get_launch_pkg(
                        "rpm_bringup",
                        "rpm_launch.py",
                    ),
                ],
                launch_arguments=get_std_lvl2_arguments(),
            ),
        ]
    )
