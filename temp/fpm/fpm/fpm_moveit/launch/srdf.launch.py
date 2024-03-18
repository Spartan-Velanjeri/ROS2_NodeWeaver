# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.actions import OpaqueFunction


def _load_srdf(context):
    # Load the SRDF robot description
    filename = os.path.join(
        get_package_share_directory('fpm_moveit'), 'srdf', 'ur.srdf')
    with open(filename, 'r') as semantic_file:
        robot_description_semantic = semantic_file.read()

    return [
        SetLaunchConfiguration(
            'robot_description_semantic', robot_description_semantic)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_load_srdf)])
