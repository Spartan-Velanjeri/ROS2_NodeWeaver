from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():


    serial_port = LaunchConfiguration(variable_name="serial_port")

    serial_port_arg = DeclareLaunchArgument(
        name="serial_port",
        default_value="/dev/ttyUSB1",
        description="USB port name assigned to the device.",
    )

    return LaunchDescription([
        serial_port_arg,
        Node(
            package='ts16',
            executable='driver',
            name='driver',
            namespace='fpm/sensors/ts16',
            output='screen',
            parameters=[
		    {"interface": "serial"},
		    {"serial_port": serial_port},
            {"serial_baud": '115200'},
		    {"logging": 'True'},
            {"mode": 'gazebo_sim'},
	    ],
        )
    ])
