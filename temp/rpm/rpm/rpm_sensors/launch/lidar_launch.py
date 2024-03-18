#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

# roe2rng (elisa.rothacker@de.bosch.com)
# This launch file launches the ouster driver
# make sure all included packages here are listed as <exec_depend> in CMakeLists.txt
# The syntax is imported from the respective launch files of the separate packages
# TODO: unify the parameter definition into one param file for readability

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------Launch Arguments-------------------------
    # mode = LaunchConfiguration('mode')
    node_name = LaunchConfiguration('name')
    driver_package_name = LaunchConfiguration('driver_package')
    executable = LaunchConfiguration('executable')
    namespace = LaunchConfiguration('namespace')
    parameter_file = LaunchConfiguration('parameters')
    params_declare = DeclareLaunchArgument(
        name='parameters',
        default_value=os.path.join(
            get_package_share_directory('rpm_sensors'), 'config', 'RPM1',
            'bautiro_ouster_os0-64.yaml'),
        description='Path to the ROS2 parameters file to use for Lidar (Ouster,...) Driver.'
    )
    mode_declare = DeclareLaunchArgument(
        name="mode",
        default_value="gazebo_sim",
        description="RPM_LOCALIZATION mode: simulation or real hardware (real, gazebo_sim)",
    )
    node_name_declare = DeclareLaunchArgument(
        name="name",
        default_value="",
        description="node_name",
    )
    package_name_declare = DeclareLaunchArgument(
        name="driver_package",
        default_value="",
        description="driver_package",
    )
    executable_declare = DeclareLaunchArgument(
        name="executable",
        default_value="",
        description="executable",
    )
    namespace_declare = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )

    # Ouster ROS2 Driver requires lifecyle nodes
    lidar_node_name = node_name
    lidar_driver_node = LifecycleNode(
        package=driver_package_name,
        executable=executable,
        name=lidar_node_name,
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ouster_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ouster_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Lidar (Ouster,...) driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # TODO make lifecycle transition to shutdown before SIGINT
    ouster_shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name=lidar_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Lidar (Ouster,...) driver node is exiting."),
            ],
        )
    )

    ld.add_action(params_declare)
    ld.add_action(mode_declare)
    ld.add_action(node_name_declare)
    ld.add_action(package_name_declare)
    ld.add_action(executable_declare)
    ld.add_action(namespace_declare)
    ld.add_action(lidar_driver_node)
    ld.add_action(ouster_activate_event)
    ld.add_action(ouster_configure_event)
    ld.add_action(ouster_shutdown_event)

    return ld
