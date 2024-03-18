# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Pytest configuration file."""

import os

import pytest
from fpm_plc_driver.ads_client import AdsClient as LiftAdsClient
from fpm_plc_driver.ads_client_ptu import AdsClient as PtuAdsClient

# from rclpy.impl.rcutils_logger import RcutilsLogger
from fpm_plc_driver.ads_logger import AdsLogger


def pytest_addoption(parser):
    parser.addoption(
        "--real_system",
        action="store",
        default=False,
        help="Enable test mode.",
    )


@pytest.fixture(scope="session")
def test_on_real_system(request):
    """Skip test if not on real system."""
    real_system = request.config.getoption("--real_system")
    print(f"Is the test running on the real system (hardware) : {real_system}")
    if not real_system:
        pytest.skip("Test is not running on the real system (hardware).")
    return real_system


@pytest.fixture(scope="session")
def ads_client():
    """Create lift ads client for the test."""

    client = LiftAdsClient(logger=AdsLogger("pytest"))
    return client


@pytest.fixture(scope="session")
def ads_ptu_client():
    """Create ptu ads client for the test."""

    client = PtuAdsClient(logger=AdsLogger("pytest"))
    return client

    # def pytest_configure(config):  # pylint: disable=unused-argument
    #     """Pre-configuration hook that runs before tests.

    #     Args:
    #         _config (pytest.Config): Pytest config.
    #     """
    #     print("\n\n********Configuration of tests.*******")
    #     print("Launching test nodes...")
    #     print("Launching lift_driver...")
    #     os.system("ros2 run fpm_plc_driver lift_driver &")
    #     # print("Launching coordinator...")
    #     # os.system("ros2 run fpm_coordinator coordinator &")

    # def pytest_unconfigure(config):  # pylint: disable=unused-argument
    #     """Post-configuration hook that runs after tests.

    #     Args:
    #         _config (pytest.Config): Pytest config.
    #     """
    #     print("********Unconfiguration of tests.*******")
    #     print("Killing test nodes...")
    #     os.system("pkill -f lift_driver")
    os.system("pkill -f coordinator")
