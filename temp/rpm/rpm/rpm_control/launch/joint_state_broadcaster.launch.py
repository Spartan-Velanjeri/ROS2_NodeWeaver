#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():
    return generate_load_controller_launch_description(
        controller_name='joint_state_broadcaster',
        controller_type='joint_state_broadcaster/JointStateBroadcaster',
        controller_params_file=os.path.join(
            get_package_share_directory('rpm_control'),
            'config', 'joint_state_broadcaster.yaml'))
