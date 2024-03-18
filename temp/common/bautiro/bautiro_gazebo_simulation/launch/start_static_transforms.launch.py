# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

standard_params = [{
    "use_sim_time": LaunchConfiguration("use_sim_time")
}]


def generate_launch_description():
    actions = []
    actions.append(Node(
        name="rpm_lidar_front_static_transform_publisher",
        package='tf2_ros',
        executable='static_transform_publisher',
        output="log",
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'rpm_lidar_front',
            'bautiro/base_link/rpm_lidar_front_lidar'],
        parameters=standard_params
    ))
    actions.append(Node(
        name="rpm_lidar_rear_static_transform_publisher",
        package='tf2_ros',
        executable='static_transform_publisher',
        output="log",
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'rpm_lidar_rear',
            'bautiro/base_link/rpm_lidar_rear_lidar'],
        parameters=standard_params
    ))
    actions.append(Node(
        name="rpm_imu_static_transform_publisher",
        package='tf2_ros',
        executable='static_transform_publisher',
        output="log",
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'rpm_imu', 'bautiro/base_link/imu_sensor'],
        parameters=standard_params
    ))
    actions.append(Node(
        name="base_link_to_ur_base_static_transform_publisher",
        package='tf2_ros',
        executable='static_transform_publisher',
        output="log",
        arguments=[
            '0.75', '-0.25', '3.186', '0', '0', '0',
            'base_link', 'ur_base_new'],
        parameters=standard_params
    ))
    return LaunchDescription(actions)
