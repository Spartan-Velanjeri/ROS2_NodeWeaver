# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

# This file is auto-generated using generate_launch2_files.py

# from slam_launch_helper import slam_launch  # modified SLAM launch helper
from ament_index_python.packages import get_package_share_directory
import os
# import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, OpaqueFunction, \
    LogInfo, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression

from slam_launch.launch_helper import slam_node_generator
# from lu_rough_localization.manipulate_yaml import change_lidar_calibration
# from lu_rough_localization.manipulate_yaml import change_imu_calibration

# def prepare_config_files():
#     cfg_file_general = os.path.join(
#         get_package_share_directory('lu_rough_localization'),
#         'config', 'fus1_slam_general.yaml')
#     cfg_file_lidar = os.path.join(
#         get_package_share_directory('lu_rough_localization'),
#         'config', 'fus1_slam_lidar.yaml')

#     # print("[RPM_LOCALIZATION_LAUNCH] Copying %s to /tmp/. ..."%(cfg_file_general))
#     # cmd_str = 'cp %s /tmp'%(cfg_file_general); os.system(cmd_str)
#     # cmd_str = 'cp %s /tmp'%(cfg_file_lidar); os.system(cmd_str)
#     # print(cmd_str)

#     # MANIPULATE HERE
#     change_imu_calibration(cfg_file_general,"/tmp/fus1_slam_general.yaml")


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'real'),
                msg=PythonExpression(expression=["'Running in ' + '",
                                                 LaunchConfiguration('mode'),
                                                 "'",
                                                 " + ' mode ...'"])))

    ld.add_action(
        LogInfo(condition=LaunchConfigurationEquals('mode', 'gazebo_sim'),
                msg=PythonExpression(expression=["'Running in ' + '",
                                                 LaunchConfiguration('mode'),
                                                 "'",
                                                 " + ' mode ...'"])))

    slam_launch_args = {
        "slam_mode": {
            "description": "The mode to run (slam, localization, lifelong_slam)",
            "default": "localization",
            "param_name": "slam_mode"
        },
        "bag": {
            "description": "The bag to run slam on (with bagmode)",
            "param_name": "bag",
            "default": ""
        },
        "dir": {
            "description": "The directory containing the bagfiles",
            "param_name": "dir",
            "default": "",
        },
        "map": {
            "description": "The map to load",
            "param_name": "map_filename",
            "default": os.path.join(
                get_package_share_directory('lu_rough_localization'),
                'maps', 'rosbag_le131')
        },
        "rs_map": {
            "description": "The road signature map to load",
            "default": "",
            "param_name": "rs_map"
        },
        "pole_map": {
            "description": "The pole map to load",
            "param_name": "pole_map_file",
            "default": ""
        },
        "lls_load_map": {
            "description": "The path where to load the lifelong slam map from",
            "param_name": "lls/map_path_loading",
            "default": ""
        },
        "lls_save_map": {
            "description": "The path to save the lifelong slam map to",
            "param_name": "lls/map_path_saving",
            "default": ""
        },
        "gt": {
            "description": "Which ground truth mode to use. Incomplete set"
                           " of options are: 'from_scan', 'ground_truth_odom'."
                           " Complete set of options depends on available plugins.",
            "param_name": "ground_truth_mode",
            "default": ""
        },
        "odo_mode": {
            "description": "The odometry mode to use",
            "default": "",
            "param_name": "odo_mode"
        },
        "prefix": {
            "description": "launch prefix (e.g., valgrind), currently used for ROS2 only",
            "default": "",
            "param_name": "prefix"
        },
        "gt_file": {
            "description": "A file containing groundtruth data.",
            "default": "",
            "param_name": "gt_file"
        },
        "mode": {
            "description": "RPM_LOCALIZATION mode: simulation or"
                           " real hardware (real, gazebo_sim)",
            "default": "gazebo_sim",
            "param_name": "real"
        },
        "hw_sample": {
            "description": "RPM_LOCALIZATION mode: simulation or "
                           "real hardware (FUS1, FUS2, ...)",
            "default": "FUS1",
            "param_name": "hw_sample"},
        "config_file_with_path": {
            "description": "(NOT USED AT THE MOMENT)",
            "default": "",
            "param_name": "config_file_with_path"},
        "namespace": {
            "description": "(NOT USED AT THE MOMENT)",
            "default": "",
            "param_name": "namespace"},
        "config_template_general": {
            "description": "config_template_general",
            "default": "",
            "param_name": "config_template_general"},
        "config_template_lidar": {
            "description": "config_template_lidar",
            "default": "",
            "param_name": "config_template_lidar"
        }
    }

    # Config file templates
    ld.add_action(DeclareLaunchArgument(
        'config_template_general',
        default_value='fus1_slam_general.yaml',
        condition=LaunchConfigurationEquals('mode', 'real')))
    ld.add_action(DeclareLaunchArgument(
        'config_template_lidar',
        default_value='fus1_slam_lidar.yaml',
        condition=LaunchConfigurationEquals('mode', 'real')))
    ld.add_action(DeclareLaunchArgument(
        'config_template_general',
        default_value='fus1_slam_general_sim.yaml',
        condition=LaunchConfigurationEquals('mode', 'gazebo_sim')))
    ld.add_action(DeclareLaunchArgument(
        'config_template_lidar',
        default_value='fus1_slam_lidar_sim.yaml',
        condition=LaunchConfigurationEquals('mode', 'gazebo_sim')))

    # manipulated files
    config_files = ["/tmp/slam_general.yaml", "/tmp/slam_lidar.yaml"]

    # TODO: Replace this for the official RewrittenYaml class
    yaml_node = Node(
        package='lu_rough_localization',
        executable='manipulate_yaml.py',
        output='both',
        parameters=[{
            'config_template_general': LaunchConfiguration('config_template_general'),
            'config_template_lidar': LaunchConfiguration('config_template_lidar'),
            'mode': LaunchConfiguration('mode')}]
    )

    # yaml_sim_node = Node(
    #       package='lu_rough_localization',
    #       executable='manipulate_yaml',
    #       output='both',
    #       condition=LaunchConfigurationEquals('mode', 'gazebo_sim'),
    #       arguments=[
    #           'yaml_file_general := "fus1_slam_general_sim.yaml"',
    #           'yaml_file_lidar   = "fus1_slam_lidar_sim.yaml"']
    # )

    # TRANSFORM_SERVICES_EXIST = 0
    TRANSFORM_SERVICES_NOT_EXIST = 256
    # ros2 service type /lu_get_trafo (Answer: bautiro_ros_interfaces/srv/GetTrafo or '')
    res = os.system('ros2 service type /lu_get_trafo')  # 256 = not found / 0 found
    transform_services = Node(
        package='lu_rough_localization',
        executable='transform_services.py',
        output='both',
        condition=IfCondition(
            PythonExpression([str(res) + " == " + str(TRANSFORM_SERVICES_NOT_EXIST)]))
    )

    for arg_name, arg_defs in slam_launch_args.items():
        ld.add_action(DeclareLaunchArgument(
            name=arg_name,
            default_value=arg_defs["default"],
            description=arg_defs["description"]
        ))

    def load_params_and_start_slam(context):
        return [slam_node_generator(context, slam_launch_args, config_files)]

    # VERSION A
    # ld.add_action(yaml_node)
    # slam_node = OpaqueFunction(function=load_params_and_start_slam)
    # ld.add_action(slam_node)

    # VERSION B: SLEEP BETWEEN EXECUTIONS
    # ld.add_action(yaml_node)
    # slam_node = OpaqueFunction(function=load_params_and_start_slam)
    # ld.add_action(TimerAction(period=5.0,actions=[slam_node]))

    # VERION C: After yaml_node is finished
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
    ld.add_action(slam_OnExecutionComplete)
    ld.add_action(transform_services)
    ld.add_action(yaml_node)
    # ld.add_action(yaml_sim_node)

    return ld

# for debuging
# generate_launch_description()
