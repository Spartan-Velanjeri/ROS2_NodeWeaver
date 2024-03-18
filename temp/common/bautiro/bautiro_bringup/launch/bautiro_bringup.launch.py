# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

# Nodes are actually deployed on the real Bautiro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration('ur_type')
    safety_limits = LaunchConfiguration('safety_limits')
    # safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    # safety_k_position = LaunchConfiguration('safety_k_position')
    # General arguments
    bautiro_description_package = LaunchConfiguration('bautiro_description_package')
    bautiro_description_file = LaunchConfiguration('bautiro_description_file')
    fpm_description_package = LaunchConfiguration('fpm_description_package')
    # fpm_description_file = LaunchConfiguration('fpm_description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    prefix = LaunchConfiguration('prefix')
    launch_rviz = LaunchConfiguration('launch_rviz')
    robot_name = LaunchConfiguration('robot_name')
    # Demo server launch file
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    # params_file = LaunchConfiguration('params_file')
    # log_level = LaunchConfiguration('log_level')
    data_service_node = Node(
        package='ccu_data_services',
        executable='data_service',
        name='data_service',
        output='screen')
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('fpm_moveit'), '/launch', '/ur_moveit.launch.py']
        ),
        launch_arguments={
            'ur_type': ur_type,
            'safety_limits': safety_limits,
            'fpm_description_package': fpm_description_package,
            'description_package': bautiro_description_package,
            'description_file': [robot_name, '/', bautiro_description_file],
            'moveit_config_package': moveit_config_package,
            'moveit_config_file': moveit_config_file,
            'prefix': prefix,
            'use_sim_time': 'true',
            'launch_rviz': launch_rviz,
        }.items(),
    )

    ccu_bt_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ccu_behavior_tree'), '/launch', '/all_trees_bringup.launch.py']
        ),
        launch_arguments={
            'namespace': namespace,
            'autostart': autostart,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    fpm_motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('fpm_cu'), '/launch', '/fpm_cu.launch.py']
        ),
        launch_arguments={
            'namespace': namespace,
            'autostart': autostart,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    data_service_node = Node(
        package='ccu_data_services',
        executable='main',
        name='data_service',
        output='screen')

    nodes_to_launch = [
        ur_moveit_launch,
        # data_service_node,
        # ccu nodes
        ccu_bt_server_launch,
        fpm_motion_manager_launch,
        data_service_node
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_type',
            description='Type/series of used UR robot.',
            choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 'ur10e', 'ur16e'],
            default_value='ur16e',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_limits',
            default_value='true',
            description='Enables the safety limits controller if true.',
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'bautiro_description_package',
            default_value='bautiro_description',
            description='Description package with robot URDF/XACRO '
                        'files. Usually the argument is not set, it '
                        'enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'bautiro_description_file',
            default_value='bautiro.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'fpm_description_package',
            default_value='fpm_description',
            description='Description package with robot URDF/XACRO'
                        ' files. Usually the argument is not set, '
                        'it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'fpm_description_file',
            default_value='ur.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='fpm_moveit',
            description='MoveIt config package with robot SRDF/XACRO files.'
                        ' Usually the argument is not set, it enables use of'
                        ' a custom moveit config.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file',
            default_value='ur.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint names, useful for multi-robot'
                        ' setup. If changed than also joint names in the '
                        'controller configuration have to be updated.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Boolean value to whether or not launch rviz',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='FUS1',
            description='Name of the robot to select configuration'
                        ' and calibration files.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='The level of logging that is applied to all '
                        'ROS 2 nodes launched by this script.',
        )
    )
    # demo launch args
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
    )
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
