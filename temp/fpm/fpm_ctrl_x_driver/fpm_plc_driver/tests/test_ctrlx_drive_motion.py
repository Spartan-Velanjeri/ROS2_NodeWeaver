# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Tests for drive in motion (emulation). DlClient (ctrl-x)"""

import time
from logging import getLogger

from fpm_plc_driver.dlClient import DlClient
from fpm_plc_driver.state import State

emulation = {
    "active": True,
    "drive_state": 5,
    "position": 100,  # position in mm
    "velocity": 0,  # velocity in mm/sec
    "velocity_profile": 80,  # velocity profile in mm/sec
}


drive = DlClient(logger=getLogger(), emulation=emulation)


def test_move():
    """Test of lift motion."""
    setpoint = emulation["position"] + 200

    drive.move_lift_absolute(position_setpoint=setpoint)

    while drive.lift_position != setpoint:
        print(f"Drive is in motion: {drive.in_motion}")
        print(f"Drive state: {drive.lift_state}")
        print(f"Lift position: {drive.lift_position}")

        assert drive.in_motion
        assert drive.lift_state == State.operation
        time.sleep(0.2)

    # target reached
    assert drive.lift_position == setpoint
    assert not drive.in_motion
    assert drive.lift_state == State.ready
