#!/bin/python3
# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep

import rclpy
from bautiro_ros_interfaces.action import MoveHandlingUnitRelativToWorkplane
from bautiro_ros_interfaces.msg import ResponseCode
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

STEPWIDTH: int = 33


class HuMoveActionServer(Node):

    def __init__(self):
        super().__init__('hu_move')

        self.current_position = [0.0, 0.0]

        # ACTIONS
        self._as = ActionServer(self,
                                MoveHandlingUnitRelativToWorkplane,
                                'fpm_set_move_relative',
                                execute_callback=self.move_hu_cb,
                                goal_callback=self.goal_validate_cb)
        self.get_logger().warn('Launch')

    def destroy_node(self):
        self._as.destroy()
        super().destroy_node()

    # ActionServer: hu_move
    def move_hu_cb(self, server_goal_handle):
        """a goal request comes in here"""

        # Read input
        goal: MoveHandlingUnitRelativToWorkplane.Goal = server_goal_handle.request
        delta_x = goal.relative_target_position[0]
        delta_y = goal.relative_target_position[1]
        self.get_logger().info(
            "Executing goal delta-'hu_move': {:5.3f}, {:5.3f}]".format(delta_x, delta_y))

        # Prepare output
        result = MoveHandlingUnitRelativToWorkplane.Result()
        feedback_msg = MoveHandlingUnitRelativToWorkplane.Feedback()

        for i in range(1, STEPWIDTH):
            if server_goal_handle.is_cancel_requested:
                server_goal_handle.canceled()
                self.get_logger().info("'hu_move' canceled: ")
                result.response_code = ResponseCode.MOVE_LIFT_ABSOLUTE_GOAL_STATUS_CANCELED
                result.current_position = self.current_position
                return result

            # update 'fake position'
            self.current_position[0] += delta_x/STEPWIDTH
            self.current_position[1] += delta_y/STEPWIDTH

            # send feedback
            feedback_msg.progress_normalized = i/STEPWIDTH
            feedback_msg.current_position = self.current_position
            x, y = self.current_position[0], self.current_position[1]
            msg = "'hu_move' progress: {:3.2f} - x:{:5.3f}, y:{:5.3f}".format(i/STEPWIDTH, x, y)
            self.get_logger().info(msg)
            server_goal_handle.publish_feedback(feedback_msg)

            sleep(3/STEPWIDTH)

        some_trouble_in_between = False
        if some_trouble_in_between:
            server_goal_handle.abort()
        else:
            server_goal_handle.succeed()
            self.get_logger().info(
                "Reached Goal 'hu_move' at: {:5.3f}, {:5.3f}".format(x, y))
        result.current_position = self.current_position
        result.response_code = ResponseCode.OKAY
        return result

    def goal_validate_cb(self, goal_request: MoveHandlingUnitRelativToWorkplane.Goal):
        """
        Whenever a client does "send_goal" - this method is called first to decide how to continue
        """
        delta_x = goal_request.relative_target_position[0]
        delta_y = goal_request.relative_target_position[1]

        if delta_x*delta_x + delta_y * delta_y > 2.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = HuMoveActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
