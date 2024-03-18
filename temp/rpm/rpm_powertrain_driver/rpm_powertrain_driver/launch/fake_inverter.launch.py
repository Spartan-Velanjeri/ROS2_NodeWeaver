#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "launch"))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    node_id_arg = DeclareLaunchArgument(
        "node_id",
        default_value=TextSubstitution(text="2"),
        description="CANopen node id the mock slave shall have.",
    )

    slave_config_arg = DeclareLaunchArgument(
        "slave_config",
        default_value=TextSubstitution(
            text=os.path.join(path_to_test, "..", "config", "fake_inverter.eds")
        ),
        description="Path to eds file to be used for the slave.",
    )

    can_interface_name_arg = DeclareLaunchArgument(
        "can_interface_name",
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface to be used by mock slave.",
    )

    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value=TextSubstitution(text="fake_canopen_inverter_node"),
        description="Name of the node.",
    )

    slave_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("node_name"),
        namespace="",
        package="rpm_powertrain_driver",
        output="screen",
        executable="fake_canopen_inverter_node",
        parameters=[
            {
                "slave_config": LaunchConfiguration("slave_config"),
                "node_id": LaunchConfiguration("node_id"),
                "can_interface_name": LaunchConfiguration("can_interface_name"),
            }
        ],
    )
    slave_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node,
            goal_state="inactive",
            handle_once=True,
            entities=[
                launch.actions.LogInfo(
                    msg="node 'fake_canopen_inverter_node' reached the 'inactive' state, 'activating'."
                ),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slave_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
    )
    slave_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld = launch.LaunchDescription()
    ld.add_action(node_id_arg)
    ld.add_action(slave_config_arg)
    ld.add_action(can_interface_name_arg)
    ld.add_action(node_name_arg)
    ld.add_action(slave_inactive_state_handler)
    ld.add_action(slave_node)
    ld.add_action(slave_configure)
    return ld
