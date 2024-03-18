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
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

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


def generate_launch_description():
    ld = LaunchDescription()

    xsens_param_file_path = Path(
        get_package_share_directory('rpm_sensors'), 'config', 'xsens_mti_node.yaml')
    xsens_mti_node = Node(
        package='bluespace_ai_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        namespace='rpm/sensors/front/imu',
        output='screen',
        parameters=[xsens_param_file_path],
        # parameters=[
        #     {"scan_for_devices": False,
        #     "port": "/dev/ttyUSB0"}
        # ],
        arguments=[]
    )

    # Ouster ROS2 Driver requires lifecyle nodes
    ouster_parameter_file = LaunchConfiguration('ouster_params_file')
    ouster_params_declare = DeclareLaunchArgument(
        'ouster_params_file',
        default_value=os.path.join(
            get_package_share_directory('rpm_sensors'), 'config', 'bautiro_ouster_os0-64.yaml'),
        description='FPath to the ROS2 parameters file to use for Ouster Driver.')
    # Ouster front
    ouster_front_node_name = 'ouster_driver'
    ouster_front_driver_node = LifecycleNode(
        package='ros2_ouster',
        executable='ouster_driver',
        name=ouster_front_node_name,
        namespace='rpm/sensors/front/lidar3d',
        output='screen',
        emulate_tty=True,
        parameters=[ouster_parameter_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ouster_front_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_front_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ouster_front_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_front_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_front_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # TODO make lifecycle transition to shutdown before SIGINT
    ouster_front_shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name=ouster_front_node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    # Ouster Rear
    ouster_rear_node_name = 'ouster_driver'
    ouster_rear_driver_node = LifecycleNode(
        package='ros2_ouster',
        executable='ouster_driver',
        name=ouster_rear_node_name,
        namespace='rpm/sensors/rear/lidar3d',
        output='screen',
        emulate_tty=True,
        parameters=[ouster_parameter_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ouster_rear_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_rear_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ouster_rear_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_rear_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_rear_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # TODO make lifecycle transition to shutdown before SIGINT
    ouster_rear_shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name=ouster_rear_node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    # urdfxacro_file_name = 'bautiro_fus1_sensor_frames_rpm.urdf.xacro'
    # xacro_path = os.path.join(
    #     get_package_share_directory('rpm_sensors'), 'config', urdfxacro_file_name)
    # robot_desc = ParameterValue(Command(['xacro ',xacro_path]), value_type=str)

    # robot_description_node = Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'robot_description': robot_desc}],
    #         # arguments=[urdf]
    # )

    ld.add_action(xsens_mti_node)
    ld.add_action(ouster_params_declare)
    ld.add_action(ouster_front_driver_node),
    ld.add_action(ouster_front_activate_event),
    ld.add_action(ouster_front_configure_event),
    ld.add_action(ouster_front_shutdown_event),
    ld.add_action(ouster_rear_driver_node),
    ld.add_action(ouster_rear_activate_event),
    ld.add_action(ouster_rear_configure_event),
    ld.add_action(ouster_rear_shutdown_event),

    return ld
