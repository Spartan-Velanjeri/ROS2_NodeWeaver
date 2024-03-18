#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import launch
import pytest
import unittest
import rclpy
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            Node(package="lifecycle", executable="lifecycle_talker", name="lc_talker"),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="test_lcm",
                output="screen",
                parameters=[
                    {"autostart": True},
                    {"node_names": ["lc_talker"]},
                ],
            ),
            ReadyToTest(),
        ]
    )


class CheckComplete(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init(args=["--ros-args", "--param", 'lifecycle_managers:=["test_lcm"]'])

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def test_complete(self):
        from bautiro_launch.check_lifecycle_managers import wait_for_active

        assert wait_for_active()
