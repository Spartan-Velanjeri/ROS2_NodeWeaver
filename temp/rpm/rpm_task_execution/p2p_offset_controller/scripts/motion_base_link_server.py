#!/bin/python3
#
#  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from tf2_ros import TransformListener, TransformStamped, TransformException
from tf2_ros.buffer import Buffer
import transformations as tf
from bautiro_ros_interfaces.action import MoveBaseLink
import math
import numpy as np


class RobotBaseController(Node):

    def __init__(self):
        super().__init__('motion_base_link_server')
        self.server = ActionServer(
            self, MoveBaseLink, 'move_base_link', self.execute_callback)
        # self.declare_parameter('use_sime_time', True)
        self.publisher = self.create_publisher(
            Twist, 'rpm_velocity_controller/cmd_vel_unstamped', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)
        self._target = Pose()
        self.current_pose = Pose()
        self.target_acquired = False
        self.control_input_linear = 0.0
        self.control_input_angular = 0.0

    # def handle_pose(self, msg):
    #     self.current_pose = msg.pose

    def execute_callback(self, goal_handle):
        # self.get_logger().info('Received goal...')

        self._target = goal_handle.request.target_pose
        self.target_acquired = True
        from_frame_rel = 'base_link'
        to_frame_rel = 'world'
        t1 = TransformStamped()
        try:
            t1 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel}'
                f' to {from_frame_rel}: {ex}')
            return

        # Convert the goal pose to Euler angles and quaternion
        euler = tf.euler_from_quaternion(
            [
                t1.transform.rotation.w,
                t1.transform.rotation.x,
                t1.transform.rotation.y,
                t1.transform.rotation.z
            ]
        )
        current_yaw = euler[-1]
        current_yaw = np.rad2deg(current_yaw)
        ex = self._target.position.x - t1.transform.translation.x
        ey = self._target.position.y - t1.transform.translation.y
        heading = np.rad2deg(math.atan2(ey, ex))
        headingError = current_yaw - heading
        orientation = tf.euler_from_quaternion(
            [
                self._target.orientation.w,
                self._target.orientation.x,
                self._target.orientation.y,
                self._target.orientation.z
            ]
        )
        desired_yaw = np.rad2deg(orientation[-1])
        self.residual = math.sqrt(ex**2 + ey**2)
        msg = Twist()
        if self.residual < 0.05:
            if abs(desired_yaw - current_yaw) > 1.0:
                self.control_input_linear = 0.0
                self.control_input_angular = (desired_yaw - current_yaw) * 0.01
                self.get_logger().info(
                    f'Orientation error:  {desired_yaw - current_yaw}')
                self.get_logger().info('____________________________')
            else:
                self.control_input_linear = 0.0
                self.control_input_angular = 0.0
                self.target_acquired = False
                self.get_logger().info(
                    f'Goal reached with Position error:  {self.residual}')
                self.get_logger().info('============================')
                self.get_logger().info('============================')
        elif abs(headingError) > 1.0 and self.residual > 0.05:
            self.control_input_linear = 0.0
            self.control_input_angular = -headingError * 0.01
            self.get_logger().info(f'Heading error:  {headingError}')
            self.get_logger().info('____________________________')
        elif abs(headingError) < 1.0 and self.residual > 0.05:
            self.control_input_linear = self.residual * 0.19
            self.control_input_angular = 0.0
            self.get_logger().info(f'Position error:  {self.residual}')
            self.get_logger().info('____________________________')

        # saturate velocities to prevent aggressive motion
        if self.control_input_linear < -0.5:
            self.control_input_linear = -0.5
        if self.control_input_linear > 0.5:
            self.control_input_linear = 0.5
        if self.control_input_angular < -0.3:
            self.control_input_angular = -0.3
        if self.control_input_angular > 0.3:
            self.control_input_angular = 0.3
        msg.linear.x = self.control_input_linear
        msg.angular.z = self.control_input_angular
        self.publisher.publish(msg)

        goal_handle.succeed()
        result_msg = MoveBaseLink.Result()
        success = Bool()

        if self.target_acquired:
            success.data = False
        else:
            success.data = True
        result_msg.pose_reached = success

        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
