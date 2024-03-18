# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition as If
from launch.substitutions import LaunchConfiguration as Lc
from launch_ros.actions import Node

__RED = '\033[91m'
__GRN = '\033[92m'
__YEL = '\033[93m'
__BLU = '\033[94m'
__MGN = '\033[95m'
__CYN = '\033[96m'
__WHT = '\033[97m'
__NOC = '\033[0m'


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('ds', default_value='True', description="False: don't start ds"),
        DeclareLaunchArgument('halc', default_value='True', description="False: don't start halc"),
        DeclareLaunchArgument('fake', default_value='True', description='False: No fake backend'),
        DeclareLaunchArgument('skill_server', default_value='False'),
        LogInfo(msg=[
            f'  {__CYN}ros2 {__BLU}launch {__RED}ccu_test.launch.py  ',
            f' {__GRN}ds{__NOC}:={__YEL}', Lc('ds'),
            f'  {__GRN}halc{__NOC}:={__YEL}',  Lc('halc'),
            f'  {__GRN}fake{__NOC}:={__YEL}',  Lc('fake'),
            f'  {__GRN}skill_server{__NOC}:={__YEL}',  Lc('skill_server'),
            f'{__NOC}',  '',
        ]),

        Node(package='ccu_data_services',
             executable='data_service',
             output='both',
             condition=If(Lc('ds'))),

        Node(package='ccu_hcu_abstraction',
             executable='halc',
             output='both',
             condition=If(Lc('halc'))),

        #
        # fake backend
        #
        Node(package='ccu_test', executable='ccu_start', output='both', condition=If(Lc('fake'))),
        Node(package='ccu_test', executable='fpm_skill', output='both', condition=If(Lc('fake'))),
        Node(package='ccu_test', executable='hu_move', output='both', condition=If(Lc('fake'))),

        #
        # skill server with dummy data
        #
        Node(package='skill_server',
             executable='skill_server',
             namespace='fpm_ccu_test',
             output='log',
             parameters=[{'demo_config': ['skill_server', 'na', 'StartFpmSkill']}],
             condition=If(Lc('skill_server'))),
    ])
