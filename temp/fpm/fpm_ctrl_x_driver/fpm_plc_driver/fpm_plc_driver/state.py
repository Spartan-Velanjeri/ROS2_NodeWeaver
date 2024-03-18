# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Definitions of the ROS node state.
"""
from enum import Enum


class State(Enum):
    """Operation states of the module."""

    not_ready = 0
    ready = 100
    operation = 200
    disabled = 300
    fault = 400
