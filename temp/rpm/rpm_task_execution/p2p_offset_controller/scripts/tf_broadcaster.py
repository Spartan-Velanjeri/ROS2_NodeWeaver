#!/usr/bin/env python3
#
#  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.


from geometry_msgs.msg import TransformStamped, PoseStamped

import rclpy
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class FramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('bautiro_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseStamped, '/reference_frame_pose', self.handle_pose, 1)
    #     self.timer = self.create_timer(1, self.broadcast_timer_callback)

    # def broadcast_timer_callback(self):
    #    t = TransformStamped()

    #    t.header.stamp = self.get_clock().now().to_msg()
    #    t.header.frame_id = 'map'
    #    t.child_frame_id = 'world'
    #    t.transform.translation.x = 0.0
    #    t.transform.translation.y = 2.0
    #    t.transform.translation.z = 0.0
    #    t.transform.rotation.x = 0.0
    #    t.transform.rotation.y = 0.0
    #    t.transform.rotation.z = 0.0
    #    t.transform.rotation.w = 1.0

    #    self.tf_broadcaster.sendTransform(t)

    def handle_pose(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        # quat = quaternion_from_euler(
        #     float(transformation[5]),
        #     float(transformation[6]),
        #     float(transformation[7]))
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.w = msg.pose.orientation.w
        self.tf_br.sendTransform(t)
        t.header.frame_id = 'world'
        t.child_frame_id = 'ur_rot'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # quat = quaternion_from_euler(
        #     float(transformation[5]),
        #     float(transformation[6]),
        #     float(transformation[7]))
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.w = msg.pose.orientation.w

        self.tf_br.sendTransform(t)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'ur_offset'
        t2.transform.translation.x = 0.75
        t2.transform.translation.y = -0.25
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t2)


def main():

    # pass parameters and initialize node
    print('start node')
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
