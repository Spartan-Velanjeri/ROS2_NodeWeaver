# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, \
    SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import yaml
import string
import random


def _configure_bridge(context, *args, **kwargs):
    """Set the correct Ignition topic source for joint states."""
    bridge_config_file = os.path.join(
        get_package_share_directory('bautiro_gazebo_simulation'),
        'config', 'bridge.yaml')

    with open(bridge_config_file, 'r') as file:
        bridge_config = yaml.safe_load(file)

    world_name = context.launch_configurations['world']
    world_name = os.path.splitext(world_name)[0]
    for i, item in enumerate(bridge_config):
        if 'ros_topic_name' in item and \
                item['ros_topic_name'] == '/joint_states':
            bridge_config[i]['ign_topic_name'] = \
                f'/world/{world_name}/model/bautiro/joint_state'

    random_string = ''.join(
        random.choices(string.ascii_letters + string.digits, k=10))
    temp_bridge_file = f'/tmp/ign_bridge_{random_string}.yaml'

    with open(temp_bridge_file, 'w') as file:
        yaml.dump(bridge_config, file)

    return [
        SetLaunchConfiguration('bridge_config_file', temp_bridge_file)
    ]


def generate_launch_description():
    actions = []
    actions.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'
        )
    )

    actions.append(
        OpaqueFunction(
            function=_configure_bridge
        )
    )

    actions.append(
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Name of the world to launch'
        )
    )

    actions.append(
        Node(
            package='ros_ign_bridge',
            executable='bridge_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'config_file': LaunchConfiguration('bridge_config_file')}
            ]
        )
    )
    return LaunchDescription(actions)
