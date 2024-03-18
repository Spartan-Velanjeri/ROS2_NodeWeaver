# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

# This file is auto-generated using generate_launch2_files.py

import os

from ament_index_python.packages import get_package_share_directory
from slam_launch.launch_helper import slam_launch


def generate_launch_description():
    return slam_launch(
        slam_launch_args={
            "slam_mode": {
                "default": "localization",
                "description": "The mode to run (slam, localization, lifelong_slam)",
                "param_name": "slam_mode"
            },
            "bag": {
                "description": "The bag to run slam on (with bagmode)",
                "default": "",
                "param_name": "bag"
            },
            "dir": {
                "description": "The directory containing the bagfiles",
                "default": "",
                "param_name": "dir"
            },
            "map": {
                "description": "The map to load",
                "param_name": "map_filename",
                "default": os.path.join(
                    get_package_share_directory('lu_rough_localization'),
                    'maps', 'rosbag_le131')
            },
            "rs_map": {
                "description": "The road signature map to load",
                "default": "",
                "param_name": "rs_map"
            },
            "pole_map": {
                "description": "The pole map to load",
                "param_name": "pole_map_file",
                "default": ""
            },
            "lls_load_map": {
                "description": "The path where to load the lifelong slam map from",
                "param_name": "lls/map_path_loading",
                "default": ""
            },
            "lls_save_map": {
                "description": "The path to save the lifelong slam map to",
                "param_name": "lls/map_path_saving",
                "default": ""
            },
            "gt": {
                "description": "Which ground truth mode to use. Incomplete set"
                               " of options are: 'from_scan', 'ground_truth_odom'."
                               " Complete set of options depends on available plugins.",
                "param_name": "ground_truth_mode",
                "default": ""
            },
            "odo_mode": {
                "description": "The odometry mode to use",
                "default": "",
                "param_name": "odo_mode"
            },
            "prefix": {
                "description": "launch prefix (e.g., valgrind), currently used for ROS2 only",
                "default": "",
                "param_name": "prefix"
            },
            "gt_file": {
                "description": "A file containing groundtruth data.",
                "default": "",
                "param_name": "gt_file"
            }
        },
        config_files=[
            '$(find lu_rough_localization)/lu_rough_localization/config/'
            'old_launch_bautiro.yaml',
            '$(find lu_rough_localization)/lu_rough_localization/config/'
            'old_launch_bautiro_vlp16.yaml'
        ]
    )
