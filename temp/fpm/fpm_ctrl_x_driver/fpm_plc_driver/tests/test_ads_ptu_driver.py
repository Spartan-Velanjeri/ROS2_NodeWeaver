# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Test of the PTU driver on the ral hardware."""

import time
from logging import DEBUG

import pytest
from fpm_plc_driver.ads_logger import get_logger

LOGGER = get_logger(__name__, DEBUG)


def test_ptu_connected(test_on_real_system, ads_ptu_client):
    """Test for connection to the plc."""
    print("\n")
    print(f"\tConnection: {ads_ptu_client.connected}")
    assert ads_ptu_client.connected


def test_ptu_rotate_left(test_on_real_system, ads_ptu_client):
    """Test for rotating left."""
    try:
        print("\n")
        print("Test for rotating left:")
        set_point = -200
        print(f"\tWriting RPM setpoint: {set_point}.")
        ads_ptu_client.rpm_set_point = set_point
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        print(f"\tReading RPM setpoint: {ads_ptu_client.rpm_set_point}.")
        assert ads_ptu_client.rpm_set_point == set_point
        time.sleep(2)
        print("\tTesting rotation...")
        assert ads_ptu_client.rpm < ads_ptu_client.rpm_set_point - 20
        print(f"\tActual rpm:{ads_ptu_client.rpm}")
        ads_ptu_client.rpm_set_point = 0
        print(f"\tRotation set to 0.")
        time.sleep(1)
        print("Done.")
    except BaseException as e:
        ads_ptu_client.rpm_set_point = 0
        raise e


def test_ptu_rotate_right(test_on_real_system, ads_ptu_client):
    """Test for rotating right."""
    try:
        print("\n")
        print("\tTest for rotating right:")
        set_point = 200
        print(f"\tWriting RPM setpoint: {set_point}.")
        ads_ptu_client.rpm_set_point = set_point
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        print(f"\tReading RPM setpoint: {ads_ptu_client.rpm_set_point}.")
        assert ads_ptu_client.rpm_set_point == set_point
        time.sleep(2)
        print("\tTesting rotation...")
        assert ads_ptu_client.rpm > ads_ptu_client.rpm_set_point - 20
        print(f"\tActual rpm:{ads_ptu_client.rpm}")
        ads_ptu_client.rpm_set_point = 0
        print(f"\tRotation set to 0.")
        time.sleep(1)
        print("Done.")
    except BaseException as e:
        ads_ptu_client.rpm_set_point = 0
        raise e


@pytest.mark.parametrize("rpm_max", [150, 250])
def test_rpm_max(rpm_max, test_on_real_system, ads_ptu_client):
    """Test for reading and writing of rpm max variable."""
    print("\n")
    print("\tTest for reading rpm max:")
    value = rpm_max
    print(f"\tWriting RPM max: {value}.")
    ads_ptu_client.rpm_max = value
    time.sleep(ads_ptu_client.read_cycle_time * 3)
    print(f"\tReading RPM max: {ads_ptu_client.rpm_max}.")
    assert ads_ptu_client.rpm_max == value
    time.sleep(2)
    print("\tTesting rotation...")
    assert ads_ptu_client.rpm > ads_ptu_client.rpm_set_point - 20


def test_led(test_on_real_system, ads_ptu_client):
    """Test for switching the led on."""
    try:
        print("\n")
        print("\tTest for switching led on/off:")
        print(f"Switching LED on.")
        ads_ptu_client.led = True
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        print(f"\tChecking led status: {ads_ptu_client.led}.")
        assert ads_ptu_client.led is True
        time.sleep(2)
        print("Switching led off.")
        ads_ptu_client.led = False
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        assert ads_ptu_client.led is False
    except BaseException as e:
        ads_ptu_client.led = False
        raise e


def test_current(test_on_real_system, ads_ptu_client):
    """Testing current read values."""
    try:
        print("\n")
        print("\tTest of current values:")
        set_point = 200
        print(f"\tWriting RPM setpoint: {set_point}.")
        ads_ptu_client.rpm_set_point = set_point
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        print(f"\tReading RPM setpoint: {ads_ptu_client.rpm_set_point}.")
        assert ads_ptu_client.rpm_set_point == set_point
        time.sleep(2)
        print("\tTesting current values...")
        print(f"\tMotor current: {ads_ptu_client.motor_current}.")
        print(f"\tAccu current: {ads_ptu_client.motor_current}.")
        assert ads_ptu_client.motor_current > 0
        assert ads_ptu_client.accu_current > 0
        ads_ptu_client.rpm_set_point = 0
        print(f"\tRotation set to 0.")
        time.sleep(1)
        print("Done.")
    except BaseException as e:
        ads_ptu_client.rpm_set_point = 0
        raise e


def test_temperature(test_on_real_system, ads_ptu_client):
    """Testing current read values."""
    try:
        print("\n")
        print("\tTest of current values:")
        set_point = 200
        print(f"\tWriting RPM setpoint: {set_point}.")
        ads_ptu_client.rpm_set_point = set_point
        time.sleep(ads_ptu_client.read_cycle_time * 3)
        print(f"\tReading RPM setpoint: {ads_ptu_client.rpm_set_point}.")
        assert ads_ptu_client.rpm_set_point == set_point
        time.sleep(2)
        print("\tTesting temperature values...")
        print(f"\tMotor temperature: {ads_ptu_client.motor_temperature}.")
        print(f"\tMcu temperature: {ads_ptu_client.mcu_temperature}.")
        assert ads_ptu_client.motor_temperature > 0
        assert ads_ptu_client.mcu_temperature > 0
        ads_ptu_client.rpm_set_point = 0
        print(f"\tRotation set to 0.")
        time.sleep(1)
        print("Done.")
    except BaseException as e:
        ads_ptu_client.rpm_set_point = 0
        raise e


@pytest.mark.parametrize(
    "command",
    [
        {"rpm_setpoint": 150, "rpm_max": 200, "asc": True, "led": True},
        {"rpm_setpoint": 250, "rpm_max": 200, "asc": False, "led": False},
        {"rpm_setpoint": -150, "rpm_max": 200, "asc": False, "led": True},
        {"rpm_setpoint": -250, "rpm_max": 200, "asc": False, "led": False},
        {"rpm_setpoint": 100, "rpm_max": 200, "asc": True, "led": True},
        {"rpm_setpoint": -100, "rpm_max": 200, "asc": True, "led": False},
    ],
)
def test_ptu_switch_on_off(command, test_on_real_system, ads_ptu_client):
    try:
        print("\n")
        print("\tTest of ptu switching on:")
        print(f"\tSwitching on with: {command}.")
        ads_ptu_client.ptu_switch_on(**command)
        time.sleep(2)
        print("\tSwitching off.")
        ads_ptu_client.ptu_switch_off()
        print("\tTesting rotation...")
        if command["rpm_setpoint"] > 0:
            assert ads_ptu_client.rpm > 0
        else:
            assert ads_ptu_client.rpm < 0
        print(f"\tActual rpm:{ads_ptu_client.rpm}")
        ads_ptu_client.rpm_set_point = 0
        print(f"\tRotation set to 0.")
        time.sleep(1)
        print("\tDone.")
    except BaseException as e:
        ads_ptu_client.rpm_set_point = 0
        ads_ptu_client.led = False
        raise e
