#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # --------------Paths--------------
    twist_goals = PathJoinSubstitution(
        [
            FindPackageShare("rpm_demo"),
            "config",
            "twist_params.yaml",
        ]
    )
    # # Rviz configuration
    # default_rviz_conf_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare("rpm_demo"),
    #         "rviz",
    #         "test_ctrl.rviz",
    #     ]
    # )


# --------------------Nodes-------------------------

    # FWD command publisher
    nodes = []
    nodes.append(
        Node(
            package="rpm_demo",
            executable="twist_message_publisher",
            parameters=[twist_goals],
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            # remappings=[
            # ('/rpm_velocity_controller/cmd_vel_unstamped', '/cmd_vel'),
            # ]
        )
    )

    # # Visualization
    # nodes.append(
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         arguments=['-d', default_rviz_conf_path],
    #     )
    # )

    return LaunchDescription(nodes)
