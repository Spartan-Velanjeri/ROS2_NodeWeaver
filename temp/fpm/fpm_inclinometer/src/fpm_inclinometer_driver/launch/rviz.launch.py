"""Launch file only for Rviz"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    rviz_description_file = PathJoinSubstitution([FindPackageShare("fpm_inclinometer_driver"),"rviz", "inclinometer.rviz"])

    ld = LaunchDescription()

    # Extension for rviz 23.09.2022
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_description_file],
    )
    
    ld.add_action(rviz_node)

    return ld
