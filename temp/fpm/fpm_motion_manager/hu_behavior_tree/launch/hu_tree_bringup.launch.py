import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
    [FindPackageShare("ur_description"), "config", "ur16e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
    [FindPackageShare("ur_description"), "config", "ur16e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
    [FindPackageShare("ur_description"), "config", "ur16e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
    [FindPackageShare("ur_description"), "config", "ur16e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "robot_ip:=192.168.1.104",
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=",
        "true",
        " ",
        "safety_pos_margin:=",
        "0.15",
        " ",
        "safety_k_position:=",
        "20",
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        "ur16e",
        " ",
        "prefix:=",
        '""',
        " ",
    ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
# MoveIt Configuration
    robot_description_semantic_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " ",
        "name:=",
        # Also ur_type parameter could be used but then the planning group names in yaml
        # configs has to be updated!
        "ur",
        " ",
        "prefix:=",
        '""',
        " ",
    ]
    )
    robot_description_semantic = {
    "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('hu_behavior_tree')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
                       'hu_bt']

    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()	
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
      
        Node(
            package='hu_behavior_tree',
            executable='hu_bt_navigators',
            output='screen',
            parameters=[robot_description,robot_description_semantic]
        ),
       
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
