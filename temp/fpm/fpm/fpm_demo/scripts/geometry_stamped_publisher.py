#!/usr/bin/env python3
# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.
#
# Author: Andreas Mogck
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped


class GeometryStampedPublisher(Node):
    def __init__(self):
        super().__init__("publisher_forward_position_controller")
        # Declare all parameters
        self.declare_parameter("wait_sec_between_publish", 10)
        self.declare_parameter("goal_names", ["pos1", "pos2"])

        # Read parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            position_name = name + '.position'
            position_desc = ParameterDescriptor(name=position_name, dynamic_typing=True)
            orientation_name = name + '.orientation'
            orientation_desc = ParameterDescriptor(name=orientation_name, dynamic_typing=True)
            self.declare_parameter(name=position_name, descriptor=position_desc)
            self.declare_parameter(name=orientation_name, descriptor=orientation_desc)
            position_values = self.get_parameter(position_name).value
            orientation_values = self.get_parameter(orientation_name).value
            goal = [position_values, orientation_values]

            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            self.goals.append(goal)

        publish_topic = "/goal_poses"

        self.get_logger().info(
            f'Publishing {len(goal_names)} goals on topic "{publish_topic}"\
              every {wait_sec_between_publish} s'
        )

        self.publisher_ = self.create_publisher(PoseStamped, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # self.get_logger().info(f'Goal id {self.goal_names[self.i]}')
        msg = PoseStamped()
        msg.pose.position.x = self.goals[self.i][0][0]
        msg.pose.position.y = self.goals[self.i][0][1]
        msg.pose.position.z = self.goals[self.i][0][2]
        msg.pose.orientation.x = self.goals[self.i][1][0]
        msg.pose.orientation.y = self.goals[self.i][1][1]
        msg.pose.orientation.z = self.goals[self.i][1][2]
        msg.pose.orientation.w = self.goals[self.i][1][3]

        self.publisher_.publish(msg)
        self.i += 1
        self.i %= len(self.goals)


def main(args=None):
    rclpy.init(args=args)

    publisher_geometry_msgs = GeometryStampedPublisher()

    rclpy.spin(publisher_geometry_msgs)
    publisher_geometry_msgs.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
