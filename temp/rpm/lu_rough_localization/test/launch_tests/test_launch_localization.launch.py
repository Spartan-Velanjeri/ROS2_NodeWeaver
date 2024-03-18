# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
# Copyright 2019 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import os
# import sys
# import unittest

# import ament_index_python

# import launch
# import launch.actions

# import launch_testing
# import launch_testing.actions
# from launch_testing.asserts import assertSequentialStdout

# import pytest


# @pytest.mark.launch_test
# def generate_test_description():
#     # This is necessary to get unbuffered output from the
#     # process under test
#     proc_env = os.environ.copy()
#     proc_env['PYTHONUNBUFFERED'] = '1'
#     # Get name of the robot
#     robot_name = os.getenv('ROBOT_NAME', 'FUS2')
#     # Launch the robot description launch file
#     robot_description_launch = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('bautiro_description'),
#                 'launch', 'robot.launch.py'])),
#         launch_arguments={
#             'use_sim_time': 'false',
#             'robot_name': robot_name}.items())

#     # Launch the localization node
#     loc_launch = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('lu_rough_localization'),
#                 'launch', 'localization_launch.py'])),
#     )

#     dut_process = launch.actions.ExecuteProcess(
#         cmd=['sleep', '10'],
#         env=proc_env, output='screen',
#         on_exit=launch.actions.Shutdown()
#     )

#     return launch.LaunchDescription([
#         robot_description_launch,
#         loc_launch,
#         # Start tests right away - no need to wait for anything
#         launch_testing.actions.ReadyToTest()]), {'dut_process': dut_process}


# @launch_testing.post_shutdown_test()
# class TestProcessOutput(unittest.TestCase):

#     def test_exit_code(self, proc_info):
#         # Check that all processes in the launch (in this case, there's just one) exit
#         # with code 0
#         launch_testing.asserts.assertExitCodes(proc_info)

#     def test_full_output(self, proc_output, dut_process):
#         # Using the SequentialStdout context manager asserts that the following stdout
#         # happened in the same order that it's checked
#         with assertSequentialStdout(proc_output, dut_process) as cm:
#             cm.assertInStdout('Starting Up')
#             for n in range(4):
#                 cm.assertInStdout('Loop {}'.format(n))
#             if os.name != 'nt':
#                 # On Windows, process termination is always forced
#                 # and thus the last print in good_proc never makes it.
#                 cm.assertInStdout('Shutting Down')

#     def test_out_of_order(self, proc_output, dut_process):
#         # This demonstrates that we notice out-of-order IO
#         with self.assertRaisesRegex(AssertionError, "'Loop 2' not found"):
#             with assertSequentialStdout(proc_output, dut_process) as cm:
#                 cm.assertInStdout('Loop 1')
#                 cm.assertInStdout('Loop 3')
#                 cm.assertInStdout('Loop 2')  # This should raise
