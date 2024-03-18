# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Test function for the DlClient (CtrlX) drive module (initialization)."""

from logging import getLogger

from fpm_plc_driver.dlClient import DlClient
from fpm_plc_driver.drive_state import DriveState
from fpm_plc_driver.state import State

emulation = {
    "active": True,
    "drive_state": 5,
    "position": 100,  # position in mm
    "velocity": 0,  # velocity in mm/sec
    "velocity_profile": 80,  # velocity profile in mm/sec
}


drive = DlClient(logger=getLogger(), emulation=emulation)


def test_is_in_emulation():
    """Test is module in emulation."""
    assert emulation["active"] == drive.emulation


def test_is_connected():
    """Test is plc connected."""
    assert emulation["active"] == drive.connected


def test_drive_state():
    """Drive state same as emulated state."""
    state = drive._drive_state

    assert state == DriveState(emulation["drive_state"])


def test_lift_state_no_motion():
    """Test state of the lift."""

    assert State.ready == drive.lift_state


def test_in_motion_not_in_motion():
    """Test is in_motion property if drive is still."""
    assert not drive.in_motion


def test_position_initial():
    """Test initial position before motion (same as in emulation parameters)."""
    assert drive.lift_position == emulation["position"]


def test_position_in_meter_initial():
    """Test lift position in meter."""
    assert drive.lift_position_meter == round(emulation["position"] / 1000, 3)


def test_lift_velocity():
    """Test of lift velocity."""

    if drive.in_motion:
        assert drive.lift_velocity > 0
        assert drive.lift_velocity_meter_sec > 0
    else:
        assert drive.lift_velocity == 0
        assert drive.lift_velocity_meter_sec == 0

    assert drive.lift_velocity == drive._emulation_parameters["velocity"]
    assert drive.lift_velocity_meter_sec == round(
        drive._emulation_parameters["velocity"] / 1000, 3
    )
