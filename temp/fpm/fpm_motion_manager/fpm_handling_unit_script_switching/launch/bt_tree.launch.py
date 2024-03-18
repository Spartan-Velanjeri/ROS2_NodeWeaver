import launch
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(get_package_share_directory('fpm_handling_unit_script_switching'), 'bt_xml')

    # Parameters
    bt_xml = LaunchConfiguration('bt_xml', default=bt_xml_dir+'/points_cluster.xml')
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
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()	
    behavior_tree = Node(
        package='fpm_handling_unit_script_switching',
        executable='bt_moveit2',
        parameters=[{'bt_xml': bt_xml},robot_description,robot_description_semantic],
        #arguments = ["--scene","/home/avp1le/Documents/139lab_robot.scene"],
        output='screen'
    )
    current_state = Node(
        package='fpm_handling_unit_script_switching',
        executable='current_state',
        parameters=[robot_description,robot_description_semantic],
        #arguments = ["--scene","/home/avp1le/Documents/139lab_robot.scene"],
        output='screen'
    )
    points = Node(
       package='fpm_cu',
       executable='load_points_service',
       #parameters=[os.path.join(get_package_share_directory('fpm_handling_unit_script_switching'),
	#                'config', 'cluster_points.yaml')],
       output='screen')
    #publish_scene = Node(
     #   package='moveit_ros_planning',
     #   executable='moveit_publish_scene_from_text',
     #   parameters=[{'bt_xml': bt_xml},robot_description,robot_description_semantic],
     #   arguments = ["--scene","/home/avp1le/Documents/139lab_robot.scene"],
     #   output='screen'
    #)
    ld = LaunchDescription()
    ld.add_action(points)
    ld.add_action(behavior_tree)
    #ld.add_action(points)
    #ld.add_action(current_state)
    return ld
