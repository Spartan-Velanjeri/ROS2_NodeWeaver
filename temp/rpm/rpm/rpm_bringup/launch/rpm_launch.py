"""Launch file for RPM bringup"""

# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os.path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from bautiro_launch.paths import get_launch_pkg
from bautiro_launch.stdargs import (
    LA_SIM_TIME,
    SIM_ONLY,
    NOT_SIM,
    LC_SIM_TIME,
    get_std_lvl1_arg_declares,
    get_std_lvl2_arguments,
)


# now we generate
def generate_launch_description():
    """Returns launch actions for RPM bringup."""
    # Not pretty, but works for now.
    nav2_dir = get_package_share_directory("rpm_nav2")
    nav2_param_file_sim = os.path.join(nav2_dir, "config", "nav2_params_sim.yaml")

    return LaunchDescription(
        get_std_lvl1_arg_declares()
        + [
            # now include the rest
            IncludeLaunchDescription(
                [
                    get_launch_pkg("bautiro_description", "robot.launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments(),
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_sensors", "rpm_sensors_launch.py"),
                ],
                condition=NOT_SIM,
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_localization", "rpm_localization_launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments(),
            ),
            Node(
                package="lu_rough_localization",
                executable="filter_temporal_spatial_outliers.py",
                name="front_lidar_filter",
                parameters=[
                    {
                        "tpc_in": "/rpm/sensors/front/lidar3d/points",
                        "max_distance": 5.0,
                        LA_SIM_TIME: LC_SIM_TIME,
                    }
                ],
                condition=NOT_SIM,
            ),
            Node(
                package="lu_rough_localization",
                executable="filter_temporal_spatial_outliers.py",
                name="rear_lidar_filter",
                parameters=[
                    {
                        "tpc_in": "/rpm/sensors/rear/lidar3d/points",
                        "max_distance": 5.0,
                        LA_SIM_TIME: LC_SIM_TIME,
                    }
                ],
                condition=NOT_SIM,
            ),
            Node(
                package="p2p_offset_controller",
                executable="controller",
                name="p2p_offset_controller",
                parameters=[{LA_SIM_TIME: LC_SIM_TIME}],
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_nav2", "navigation_launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments(),
                condition=NOT_SIM,
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_nav2", "navigation_launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments()
                + [("params_file", nav2_param_file_sim)],
                condition=SIM_ONLY,
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_behavior_tree", "rpm_tree_bringup.launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments(),
            ),
            IncludeLaunchDescription(
                [
                    get_launch_pkg("rpm_nav2", "map_launch.py"),
                ],
                launch_arguments=get_std_lvl2_arguments(),
            ),
        ]
    )
