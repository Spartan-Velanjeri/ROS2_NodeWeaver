#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from time import sleep

from rclpy import init, shutdown, spin
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from bautiro_ros_interfaces.action import StartFpmSkill

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG


class FpmSkillTestBackend(Node):

    def __init__(self):
        super().__init__('fpm_skill_test_backend')
        self.s = ActionServer(self, StartFpmSkill, '/fpm_skill', self.do_it, callback_group=CbG())
        self.get_logger().warn('Launch')

    def destroy_node(self):
        self.s.destroy()
        super().destroy_node()

    def do_it(self, server_goal_handle: ServerGoalHandle):
        """Goal request comes in here."""
        goal: StartFpmSkill.Goal = server_goal_handle.request
        self.get_logger().info(f'GOAL - Start Work           : {goal.skill_name}')
        # Read input
        # we are doing nothing with the 'input' goal.start = True
        sleep(0.999)
        some_trouble_in_between = False
        if some_trouble_in_between:
            server_goal_handle.abort()
            self.get_logger().info('Abort Goal')
        else:
            server_goal_handle.succeed()
            self.get_logger().info(f'GOAL - Reached Successfully : {goal.skill_name}')

        r = StartFpmSkill.Result()
        r.response_code = 0
        self.get_logger().info(f'GOAL - Return Result Here   : {goal.skill_name}')
        return r


def main():
    init()
    node = FpmSkillTestBackend()
    spin(node)
    node.destroy_node()
    shutdown()


if __name__ == '__main__':
    main()
