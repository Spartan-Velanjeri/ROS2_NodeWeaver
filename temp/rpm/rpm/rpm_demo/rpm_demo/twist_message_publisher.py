#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class TwistMessagePublisher(Node):

    def __init__(self):
        super().__init__(node_name="twist_message_publisher")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('controller_name', "rpm_velocity_controller"),
                ('wait_sec_between_publish', 5),
                ('goal_names', ["twist_1", "twist_2"]),
                # ('twist_1.linear', [10.0, 0.0, 0.0]),
                # ('twist_1.angular', [0.0, 0.0, 0.0]),
                # ('twist_2.linear',  [0.0, 0.0, 0.0]),
                # ('twist_2.angular', [0.0, 0.0, 0.5]),
            ])

        # Read parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        controller_name = self.get_parameter("controller_name").value
        goal_names = self.get_parameter("goal_names").value

        # Read twists from parameters

        self.twists = []

        for twist_n in goal_names:
            linear_name = twist_n + '.linear'
            linear_desc = ParameterDescriptor(name=linear_name, dynamic_typing=True)
            angular_name = twist_n + '.angular'
            angular_desc = ParameterDescriptor(name=angular_name, dynamic_typing=True)
            self.declare_parameter(name=linear_name, descriptor=linear_desc)
            self.declare_parameter(name=angular_name, descriptor=angular_desc)
            vector_linear = self.get_parameter(linear_name).value
            vector_angular = self.get_parameter(angular_name).value
            twist = [vector_linear, vector_angular]
            self.twists.append(twist)

        publish_topic = "/" + controller_name + "/" + "cmd_vel_unstamped"

        self.get_logger().info(
            f'Publishing {len(goal_names)} twists on topic "{publish_topic}" '
            f'every {wait_sec_between_publish} s'
        )

        self.publisher = self.create_publisher(Twist, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg: Twist = Twist()  # self.get_parameter("myTwistKey").value
        msg.linear.x = x = self.twists[self.i][0][0]
        msg.linear.y = y = self.twists[self.i][0][1]
        msg.linear.z = z = self.twists[self.i][0][2]
        msg.angular.x = roll = self.twists[self.i][1][0]
        msg.angular.y = pitch = self.twists[self.i][1][1]
        msg.angular.z = yaw = self.twists[self.i][1][2]

        self.get_logger().info(
            f'Publishing [Twist] -linear-  x: {x}, y: {y}, z: {z} '
            f' -angular- x: {roll}, y: {pitch}, z: {yaw}')
        self.publisher.publish(msg)
        self.i += 1
        self.i %= len(self.twists)


def main(args=None):
    rclpy.init(args=args)
    node = TwistMessagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
