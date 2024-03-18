# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, \
    OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_description(context, *args, **kwargs):
    """Load robot description."""
    robot_name = context.launch_configurations['robot_name']

    urdf_filename = os.path.join(
        get_package_share_directory('bautiro_description'), 'robots',
        robot_name, 'bautiro.urdf')

    if not os.path.isfile(urdf_filename):
        raise FileNotFoundError(f'File {urdf_filename} not found')

    if os.stat(urdf_filename).st_size == 0:
        raise OSError(f'File {urdf_filename} is empty')

    with open(urdf_filename, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return [
        SetLaunchConfiguration(
            'robot_description', robot_description)]


def generate_launch_description():
    actions = []
    # Declare input arguments
    actions.append(
        DeclareLaunchArgument(
            'robot_name',
            description='Name of the robot to select configuration '
                        'and calibration files.')
    )

    # Set opaque function to parse the robot description
    actions.append(OpaqueFunction(function=_load_description))

    # Store the robot description after parsing the URDF file
    robot_description = {
        'robot_description': LaunchConfiguration('robot_description')}

    # Robot state publisher
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="log",
        arguments=[('__log_level:=debug')],
        # IMPORTANT: DO NOT CONFIGURE THIS NODE WITH SIM_TIME, GAZEBO DEPENDS ON IT
        parameters=[robot_description])
    )

    return LaunchDescription(actions)
