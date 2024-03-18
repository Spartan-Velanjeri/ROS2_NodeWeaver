#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

import sys

import rclpy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FramePublisher(Node):
    def __init__(self):
        super().__init__('slam_mockup')

        self._tf_publisher = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):        
        self.make_transforms_quat('map', 'leica',-10.0,0.0,0.5,0.0,0.0,0.0,1.0)
        

    def make_transforms(self, parent_frame_id, frame_id, x_trans, y_trans, z_trans, roll, pitch, yaw):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = parent_frame_id
        static_transformStamped.child_frame_id = frame_id
        static_transformStamped.transform.translation.x = x_trans
        static_transformStamped.transform.translation.y = y_trans
        static_transformStamped.transform.translation.z = z_trans
        quat = tf_transformations.quaternion_from_euler(
            roll, pitch, yaw)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

    def make_transforms_quat(self, parent_frame_id, frame_id, x_trans, y_trans, z_trans, quat_x, quat_y, quat_z, quat_w):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = parent_frame_id
        static_transformStamped.child_frame_id = frame_id
        static_transformStamped.transform.translation.x = x_trans
        static_transformStamped.transform.translation.y = y_trans
        static_transformStamped.transform.translation.z = z_trans
        static_transformStamped.transform.rotation.x = quat_x
        static_transformStamped.transform.rotation.y = quat_y
        static_transformStamped.transform.rotation.z = quat_z
        static_transformStamped.transform.rotation.w = quat_w

        self._tf_publisher.sendTransform(static_transformStamped)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
