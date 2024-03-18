import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    return LaunchDescription([
        Node(
            package='fpm_handling_unit_script_switching',
            executable='load_points',
            name='parameter_types_example',
            parameters=[os.path.join(
                get_package_share_directory('fpm_handling_unit_script_switching'),
                'config', 'cluster_points.yaml')],
            output='screen'),

    ])
