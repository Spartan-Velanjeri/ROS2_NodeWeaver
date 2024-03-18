# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import sys

import rclpy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('tf_reflex_marker_broadcaster')

        self._tf_publisher = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        # read coordinate of all reflex markers
        # Publish static transforms once at startup
        self.make_transforms('digipas_inclinometer_base','dis_sensor_inclinometer_base', 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

#  ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 1  

# Offset der beiden Koordinatensystem  --> bei Aufruf??
# Wo bleibt der "static_transform_publisher"

    def make_transforms(self, frame_id, child_frame_id, x_trans, y_trans, z_trans, roll, pitch, yaw):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = frame_id
        static_transformStamped.child_frame_id = child_frame_id
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

def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()