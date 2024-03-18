# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
from ament_index_python.packages import get_package_share_directory

from controller_manager.launch_utils import generate_load_controller_launch_description

import os


def generate_launch_description():
    return generate_load_controller_launch_description(
        controller_name='lift_controller',
        controller_type='lift_command_controller/LiftCommandController',
        controller_params_file=os.path.join(
            get_package_share_directory('fpm_control'),
            'config', 'lifting_unit_controller.yaml'))
