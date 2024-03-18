# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Test for testing the motion of the lift on the real system."""
import time

import pytest


def test_connected(test_on_real_system, ads_client):
    """Test for connection to the plc."""
    print("\n")
    print(f"\tConnection: {ads_client.connected}")
    assert ads_client.connected


def test_velocity_profile_read(test_on_real_system, ads_client):
    """Test reading of the velocity profile."""
    print(f"\tVelocity profile [mm/sec]: {ads_client.velocity_profile}")
    assert isinstance(ads_client.velocity_profile, float)  # it should be not 0


def test_velocity_profile_write(test_on_real_system, ads_client):
    """Test for writing the velocity profile"""
    velocity_profile = ads_client.velocity_profile
    if velocity_profile != 50:
        new_velocity_profile = 50
    else:
        new_velocity_profile = 40
    ads_client.set_velocity_profile(velocity_profile=new_velocity_profile)
    print(f"\tNew velocity profile [mm/sec]: {ads_client.velocity_profile}")
    assert ads_client.velocity_profile == new_velocity_profile


@pytest.mark.skip(reason="Not implemented")
def test_enable_lift(test_on_real_system):
    """Test for enabling the lift."""
    pass


@pytest.mark.skip(reason="Not implemented")
def test_disable_lift(test_on_real_system):
    """Test for disabling the lift."""
    pass


@pytest.mark.skip(reason="How to implement?")
def test_reset_lift_error():
    """Test for disabling the lift."""
    pass


def test_motion(test_on_real_system, ads_client):
    """Testing of motion driver parameters and functions:
    * in_motion - flag
    * move_lift_absolute - fcn
    * lift_position - parameter
    * lift_position_meter - parameter
    """

    const_velocity_profile = 5

    # use small velocity
    ads_client.set_velocity_profile(const_velocity_profile)
    print(f"\tNew velocity profile [mm/sec]: {ads_client.velocity_profile}")
    current_position = ads_client.lift_position
    print(f"\tCurrent position [mm]: {current_position}")
    if current_position > 50:
        new_position = current_position - 50
    else:
        new_position = current_position + 50
    ads_client.move_lift_absolute(position_setpoint=new_position)
    print(f"\tMoving to position [mm]: {new_position}")

    time.sleep(0.5)

    assert ads_client.in_motion

    while ads_client.in_motion:
        time.sleep(0.5)
        print(f"\tVelocity [mm/sec]: {ads_client.lift_velocity}")
        # assert client.lift_velocity == const_velocity_profile

    print(f"\tNew position [mm]: {ads_client.lift_position}")
    assert ads_client.lift_position == new_position
    print(f"\tNew position [m]: {ads_client.lift_position_meter}")
    assert ads_client.lift_position_meter == new_position / 1000


def test_motion_stop(test_on_real_system, ads_client):
    """Testing of motion stopping."""

    const_velocity_profile = 5

    # use small velocity
    ads_client.set_velocity_profile(const_velocity_profile)
    print(f"\tNew velocity profile [mm/sec]: {ads_client.velocity_profile}")
    current_position = ads_client.lift_position
    print(f"\tCurrent position [mm]: {current_position}")
    if current_position > 50:
        new_position = current_position - 50
    else:
        new_position = current_position + 50
    ads_client.move_lift_absolute(position_setpoint=new_position)
    print(f"\tMoving to position [mm]: {new_position}")

    # motion takes about 10 sec
    time.sleep(2)
    assert ads_client.in_motion

    while ads_client.in_motion:
        ads_client.stop_lift()

    time.sleep(0.5)
    assert not ads_client.in_motion
    print(f"\tNew position after stop [mm]: {ads_client.lift_position}")
