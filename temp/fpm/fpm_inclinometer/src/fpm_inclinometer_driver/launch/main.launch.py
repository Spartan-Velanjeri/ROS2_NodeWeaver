"""Inclinometer launch file"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("fpm_inclinometer_driver"),
        'config',
        'params.yaml'
        )

    digipas_node = Node(
        package="fpm_inclinometer_driver",
        executable="digipas_driver_node",
        parameters=[config]
    ) 
    
    # Extension for transform_publisher 23.09.2022
    transform_node = Node(
        package="fpm_inclinometer_driver",
        executable="inclinometer_transform_publisher_node"
    )
    
    ld.add_action(digipas_node)
    ld.add_action(transform_node)

    return ld
