# Copyright 2021 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Nodes ############################################################
    # TRANSFORM_SERVICES_EXIST = 0
    # TRANSFORM_SERVICES_NOT_EXIST = 256
    # ros2 service type /lu_get_trafo (Answer: bautiro_ros_interfaces/srv/GetTrafo or '')
    # res = os.system("ros2 service type /lu_get_trafo")  # 256 = not found / 0 found
    # transform_srv = Node(
    #     package="lu_rough_localization",
    #     executable="transform_services",
    #     output="both",
    #     condition=IfCondition(
    #         PythonExpression([str(res) + " == " + str(TRANSFORM_SERVICES_NOT_EXIST)])
    #     ),
    # )

    fine_loc_srv = Node(
        package="lu_fine_localization",
        executable="fine_localization_service.py",
        output="both",
    )

    # Other LAUNCH files ###############################################
    # fpm_sensors_dir = get_package_share_directory("fpm_sensors")
    # fpm_sensors_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(fpm_sensors_dir + '/launch/fpm_sensors_launch.py'))

    # Add actions #######################################################
    # ld.add_action(fpm_sensors_launch)
    # ld.add_action(transform_srv)
    ld.add_action(fine_loc_srv)

    return ld
