# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package='rosbridge_server',
             executable='rosbridge_websocket',
             output='screen',
             parameters=[{'port': 9090},
                         {'address': ''},
                         {'retry_startup_delay': 5.0},
                         {'fragment_timeout': 600},
                         {'delay_between_messages': 0},
                         {'max_message_size': 10000000},
                         {'unregister_timeout': 10.0},
                         {'use_compression': False},
                         {'topics_glob': '[*]'},
                         {'services_glob': '[*]'},
                         {'params_glob': '[*]'},
                         {'bson_only_mode': False}]),
        Node(package='rosapi',
             executable='rosapi_node',
             parameters=[{'topics_glob': '[*]'},
                         {'services_glob': '[*]'},
                         {'params_glob': '[*]'}])
    ])
