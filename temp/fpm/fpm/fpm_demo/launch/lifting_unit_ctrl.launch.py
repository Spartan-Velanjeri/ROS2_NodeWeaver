# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --------------Paths--------------
    # Position goals
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("fpm_demo"),
            "config",
            "listing_unit_absolut_positions.yaml",
        ]
    )
    # Rviz configuration
    default_rviz_conf_path = PathJoinSubstitution(
        [
            FindPackageShare("fpm_control"),
            "rviz",
            "test_ctrl.rviz",
        ]
    )

    # --------------------Nodes-------------------------

    # FWD command publisher
    nodes = []
    nodes.append(Node(
        package="ros2_control_test_nodes",
        executable="publisher_forward_position_controller",
        name="publisher_forward_position_controller",
        parameters=[position_goals],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        # remappings=[
        #   ('/forward_position_controller/commands', '/position_commands'),
        # ]
    )
    )

    # Visualization
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_conf_path],
        )
    )

    return LaunchDescription(nodes)
