# Copyright 2021 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

# This file is auto-generated using generate_launch2_files.py

# from slam_launch_helper import slam_launch  # modified SLAM launch helper
from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression

from lu_rough_localization.slam_launch_helper import slam_node_generator
from rclpy.logging import get_logger


LOGGER = get_logger(__name__)


def config_template_launch_setup(context, *args, **kwargs):
    launch_args = []

    # config file with path
    config_file_with_path = \
        LaunchConfiguration("config_file_with_path").perform(context)
    with open(config_file_with_path, 'r') as cf:
        config_rpm_txt = yaml.load(cf, Loader=yaml.SafeLoader)

    slam_general = \
        config_rpm_txt['config_template_general']['mode']['real']
    slam_general_sim = \
        config_rpm_txt['config_template_general']['mode']['gazebo_sim']
    slam_lidar = \
        config_rpm_txt['config_template_lidar']['mode']['real']
    slam_lidar_sim = \
        config_rpm_txt['config_template_lidar']['mode']['gazebo_sim']

    launch_args.append(
        DeclareLaunchArgument(
            'config_template_general',
            default_value=slam_general,
            condition=LaunchConfigurationEquals('mode', 'real')))
    launch_args.append(
        DeclareLaunchArgument(
            'config_template_lidar',
            default_value=slam_lidar,
            condition=LaunchConfigurationEquals('mode', 'real')))
    launch_args.append(
        DeclareLaunchArgument(
            'config_template_general',
            default_value=slam_general_sim,
            condition=LaunchConfigurationEquals('mode', 'gazebo_sim')))
    launch_args.append(
        DeclareLaunchArgument(
            'config_template_lidar',
            default_value=slam_lidar_sim,
            condition=LaunchConfigurationEquals('mode', 'gazebo_sim')))

    return launch_args


def launch_setup(context, *args, **kwargs):
    launch_descriptions = []

    # manipulated files
    config_files = ["/tmp/slam_general.yaml", "/tmp/slam_lidar.yaml"]

    # config file with path
    config_file_with_path = LaunchConfiguration("config_file_with_path").perform(context)

    # slam_launch_args file path
    slam_launch_args_file = os.path.join(
        os.path.dirname(config_file_with_path), 'slam_launch_args.yaml')
    LOGGER.info(f"used slam launch args file: {slam_launch_args_file} ")

    # read slam launch args file, DeclareLaunchArguments of slam respective hw_sample
    with open(slam_launch_args_file, 'r') as sf:
        slam_launch_args_text = yaml.load(sf, Loader=yaml.SafeLoader)
        for slam_arg, params in slam_launch_args_text.items():
            launch_descriptions.append(DeclareLaunchArgument(
                name=slam_arg,
                default_value=params["default"],
                description=params["description"]
            ))

    def load_params_and_start_slam(context):
        return [slam_node_generator(context, slam_launch_args_text, config_files)]

    yaml_node = Node(
        package='lu_rough_localization',
        executable='manipulate_yaml.py',
        output='both',
        parameters=[{
            'config_template_general': LaunchConfiguration('config_template_general'),
            'config_template_lidar': LaunchConfiguration('config_template_lidar'),
            'mode': LaunchConfiguration('mode')
        }]
    )

    # TRANSFORM_SERVICES_EXIST = 0
    TRANSFORM_SERVICES_NOT_EXIST = 256
    # ros2 service type /lu_get_trafo (Answer: bautiro_ros_interfaces/srv/GetTrafo or '')
    res = os.system('ros2 service type /lu_get_trafo')  # 256 = not found / 0 found
    transform_services = Node(
        package='lu_rough_localization',
        executable='transform_services.py',
        output='log',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(
            PythonExpression([str(res) + " == " + str(TRANSFORM_SERVICES_NOT_EXIST)]))
    )

    # After yaml_node is finished
    slam_node = OpaqueFunction(function=load_params_and_start_slam)
    slam_OnExecutionComplete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=yaml_node,
            on_completion=[
                LogInfo(msg='Starting SLAM ...'),
                slam_node
            ]
        )
    )

    # define the OnExecutionComplete condition before yaml_node starting!!!
    launch_descriptions.append(slam_OnExecutionComplete)
    launch_descriptions.append(transform_services)
    launch_descriptions.append(yaml_node)

    return launch_descriptions


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "hw_sample",
            default_value="RPM1",
            description='Name of the Hardware Sample',
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "mode",
            default_value="gazebo_sim",
            description='Name of the mode (real, gazebo_sim)',
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "config_file_with_path",
            default_value=os.path.join(
                get_package_share_directory('rpm_localization'), 'config',
                'RPM1', 'rpm_config.yaml'),
            description='Name of config file',
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description='Namespace',
        )
    )

    ld.add_action(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'real'),
                msg=PythonExpression(expression=[
                    "'Running in ' + '",
                    LaunchConfiguration('mode'),
                    "'",
                    " + ' mode ...'"])
                )
    )

    ld.add_action(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'gazebo_sim'),
                msg=PythonExpression(expression=[
                    "'Running in ' + '",
                    LaunchConfiguration('mode'),
                    "'",
                    " + ' mode ...'"])
                )
    )

    # Load config file templates
    ld.add_action(OpaqueFunction(function=config_template_launch_setup))

    # get SLAM launch args and add SLAM, transform_services and yaml_node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
