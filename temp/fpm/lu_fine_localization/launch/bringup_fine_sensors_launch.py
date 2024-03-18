# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
#
# bes2rng (stefan.benz@de.bosch.com)
# This launch file launches all drivers for the bautiro localization waegele
# make sure all included packages here are listed as <exec_depend> in CMakeLists.txt
# The syntax is imported from the respective launch files of the seperate packages
# TODO: unify the parameter definition into one param file for readability

# flake8: noqa

from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
import os
#from ouster driver launch file

from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    ld = LaunchDescription()

    leica_node = Node(
        package="ts60",
        executable="driver",
        name="driver",
        namespace="fine/ts60",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"tcp_ip": "192.168.254.3"},
            {"tcp_port": "1212"},
            {"interface": "usb"},
            {"logging": "False"},
        ]
    )

    ld.add_action(leica_node)

    return ld
