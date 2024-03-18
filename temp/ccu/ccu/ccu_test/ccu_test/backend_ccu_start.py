#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from time import sleep
from rclpy import init, shutdown, spin
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from bautiro_ros_interfaces.action import StartCcuBt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CgB


class CcuStart(Node):

    def __init__(self):
        super().__init__('fake_backend_ccu_start_node')
        self._as = ActionServer(self, StartCcuBt, 'ccu_start',
                                execute_callback=self.do_it,
                                goal_callback=self.validate_goal,
                                callback_group=CgB())
        self.get_logger().warn('Launch')

    def destroy_node(self):
        self._as.destroy()
        super().destroy_node()

    def validate_goal(self, goal_request: StartCcuBt.Goal):
        """When client does `send_goal` - this is called first to block or allow."""
        if goal_request.start:
            self.get_logger().info("Goal Accepted")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info("Goal Rejected (don't call with 'start:False')")
            return GoalResponse.REJECT

    def do_it(self, server_goal_handle):
        """Goal request comes in here."""
        # Read input
        # goal: StartCcuBt.Goal = server_goal_handle.request
        # we are doing nothing with the 'input' goal.start = True
        self.get_logger().info("Do some Work - Start")
        sleep(0.5)
        self.get_logger().info("Do some Work - Done")

        some_trouble_in_between = False
        if some_trouble_in_between:
            server_goal_handle.abort()
            self.get_logger().info('Abort Goal')
        else:
            server_goal_handle.succeed()
            self.get_logger().info('Reached Goal : set Status to SUCCEED')
        result = StartCcuBt.Result()
        result.message = 'hello'
        self.get_logger().info('Reached Goal: Return Result here')
        return result


def main():
    init()
    node = CcuStart()
    spin(node)
    node.destroy_node()
    shutdown()


if __name__ == '__main__':
    main()
