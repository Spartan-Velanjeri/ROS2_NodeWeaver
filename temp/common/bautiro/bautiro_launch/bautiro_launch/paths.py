"""Provide standard arguments that we use in all our launch files."""
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os.path
from ament_index_python.packages import get_package_share_directory


# utility functions
def get_launch_pkg(package_name: str, launch_filename: str):
    """Return absolute launch filename from pkg and filename."""
    return os.path.join(
        get_package_share_directory(package_name), "launch", launch_filename
    )
