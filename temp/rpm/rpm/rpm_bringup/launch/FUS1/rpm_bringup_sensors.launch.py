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

    # sick_front_left_node = Node(
    #     package="sick_safetyscanners2",
    #     executable="sick_safetyscanners2_node",
    #     name="sick_safetyscanners2_node",
    #     namespace="rough/front_left/sick",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[
    #         {"frame_id": "sick_front_left",
    #             "sensor_ip": "192.168.13.101",
    #             "host_ip": "192.168.13.1",
    #             "interface_ip": "0.0.0.0",
    #             "host_udp_port": 0,
    #             "channel": 0,
    #             "channel_enabled": True,
    #             "skip": 0,
    #             "angle_start": 0.0,
    #             "angle_end": 0.0,
    #             "time_offset": 0.0,
    #             "general_system_state": True,
    #             "derived_settings": True,
    #             "measurement_data": True,
    #             "intrusion_data": True,
    #             "application_io_data": True,
    #             "use_persistent_config": False,
    #             "min_intensities": 0.0
    #         }
    #     ]
    # )

    # sick_rear_right_node = Node(
    #     package="sick_safetyscanners2",
    #     executable="sick_safetyscanners2_node",
    #     name="sick_safetyscanners2_node",
    #     namespace="rough/rear_right/sick",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[
    #         {"frame_id": "sick_rear_right",
    #             "sensor_ip": "192.168.14.101",
    #             "host_ip": "192.168.14.1",
    #             "interface_ip": "0.0.0.0",
    #             "host_udp_port": 0,
    #             "channel": 0,
    #             "channel_enabled": True,
    #             "skip": 0,
    #             "angle_start": 0.0,
    #             "angle_end": 0.0,
    #             "time_offset": 0.0,
    #             "general_system_state": True,
    #             "derived_settings": True,
    #             "measurement_data": True,
    #             "intrusion_data": True,
    #             "application_io_data": True,
    #             "use_persistent_config": False,
    #             "min_intensities": 0.0
    #         }
    #     ]
    # )

    xsens_param_file_path = Path(
        get_package_share_directory('rpm_bringup'), 'config', 'FUS1', 'xsens_mti_node.yaml')
    xsens_mti_node = Node(
        package='bluespace_ai_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        namespace='rpm/front_right/xsens',
        output='screen',
        parameters=[xsens_param_file_path],
        arguments=[]
    )

    # Ouster ROS2 Driver requires lifecyle nodes
    ouster_parameter_file = LaunchConfiguration('ouster_params_file')
    ouster_params_declare = DeclareLaunchArgument(
        'ouster_params_file',
        default_value=os.path.join(
            get_package_share_directory('rpm_bringup'), 'config',
            'FUS1', 'bautiro_ouster_os0-64.yaml'),
        description='Path to the ROS2 parameters file to use for Ouster Driver.')
    # Ouster Front Right
    # ouster_FR_node_name = 'ouster_driver'
    # ouster_FR_driver_node = LifecycleNode(package='ros2_ouster',
    #                             executable='ouster_driver',
    #                             name=ouster_FR_node_name,
    #                             namespace='rough/front_right/ouster',
    #                             output='screen',
    #                             emulate_tty=True,
    #                             parameters=[ouster_parameter_file],
    #                             arguments=['--ros-args', '--log-level', 'INFO'],
    #                             )
    # ouster_FR_configure_event = EmitEvent(
    #     event=ChangeState(
    #         lifecycle_node_matcher=matches_action(ouster_FL_driver_node),
    #         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    #     )
    # )

    # Ouster Front Left
    ouster_FL_node_name = 'ouster_driver'
    ouster_FL_driver_node = LifecycleNode(
        package='ros2_ouster',
        executable='ouster_driver',
        name=ouster_FL_node_name,
        namespace='rough/front_left/ouster',
        output='screen',
        emulate_tty=True,
        parameters=[ouster_parameter_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ouster_FL_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_FL_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ouster_FL_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_FL_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_FL_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # TODO make lifecycle transition to shutdown before SIGINT
    ouster_FL_shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name=ouster_FL_node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    # Ouster Rear Right
    ouster_RR_node_name = 'ouster_driver'
    ouster_RR_driver_node = LifecycleNode(
        package='ros2_ouster',
        executable='ouster_driver',
        name=ouster_RR_node_name,
        namespace='rough/rear_right/ouster',
        output='screen',
        emulate_tty=True,
        parameters=[ouster_parameter_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ouster_RR_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_RR_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ouster_RR_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_RR_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_RR_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # TODO make lifecycle transition to shutdown before SIGINT
    ouster_RR_shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name=ouster_RR_node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    # urdf
    # urdfxacro_file_name = 'bautiro_waegele_sensor_frames.urdf.xacro'
    # xacro_path = os.path.join(
    #     get_package_share_directory('lu_waegele'), 'urdf', urdfxacro_file_name)
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
    ld.add_action(ouster_FL_driver_node),
    ld.add_action(ouster_FL_activate_event),
    ld.add_action(ouster_FL_configure_event),
    ld.add_action(ouster_FL_shutdown_event),
    ld.add_action(ouster_RR_driver_node),
    ld.add_action(ouster_RR_activate_event),
    ld.add_action(ouster_RR_configure_event),
    ld.add_action(ouster_RR_shutdown_event),

    return ld
