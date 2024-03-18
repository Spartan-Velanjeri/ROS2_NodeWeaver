#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# bes2rng (stefan.benz@de.bosch.com)
# This launch file launches all drivers for the bautiro localization waegele
# make sure all included packages here are listed as <exec_depend> in CMakeLists.txt
# The syntax is imported from the respective launch files of the seperate packages
# TODO: unify the parameter definition into one param file for readability

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    leica_node = Node(
        package="ts16",
        executable="driver",
        name="driver",
        namespace="fine/ts16",
        output="screen",
        parameters=[
            # ATTENTION: PARAMETER HAVE NO IMPACT HERE!!!!!!!!!!
            {"serial_port": "/dev/ttyUSB1"},
            {"serial_baud": "115200"},
            {"interface": "serial"},
            {"logging": "False"}
        ]
    )

    ld.add_action(leica_node)

    return ld
