# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import rclpy
import numpy as np
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf_transformations import quaternion_matrix, \
    quaternion_from_matrix

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LeicaPosepublisher(Node):

    def __init__(self):
        super().__init__('leica_pose_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Odometry, 'pks', 10)

        # Listens to odom data from world frame to  base_link
        self.subscription_world_to_base_link = self.create_subscription(
            Odometry,
            'odometry',
            self.listener_callback,
            10)

        # Calculation of transform from world to lift_top
        self.transform_callback_base_link_to_lift_top = self.create_timer(
            0.1, self.transform_callback)

        # Publisher to publish the translation and rotation values to a topic
        self.publisher_timer_callback = self.create_timer(
            0.1, self.publisher_callback)

    def listener_callback(self, msg):
        # (roll, pitch, yaw) = euler_from_quaternion(
        #       [msg.pose.pose.orientation.x,
        #        msg.pose.pose.orientation.y,
        #        msg.pose.pose.orientation.z,
        #        msg.pose.pose.orientation.w])
        # self.get_logger().info(
        #   f'Subscribing -> Linear X : "{msg.pose.pose.position.x}",
        #   Linear Y : "{msg.pose.pose.position.y}",
        #   Linear Z : "{msg.pose.pose.position.z}" and Angular Z : "{yaw}"')

        # Creating a transformation matrix out of the
        # translation and orientation values
        self.tf_matrix_odom_to_base_link = quaternion_matrix(
            [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])
        self.tf_matrix_odom_to_base_link[0][3] = msg.pose.pose.position.x
        self.tf_matrix_odom_to_base_link[1][3] = msg.pose.pose.position.y
        self.tf_matrix_odom_to_base_link[2][3] = msg.pose.pose.position.z

    def transform_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link",
                "lift_top",
                rclpy.time.Time())
            # self.get_logger().info(f'X : "{t.transform.translation.x}" ,Y : "{t.transform.translation.y}" ,Z : "{t.transform.translation.z}"')  # noqa
        except TransformException as ex:
            self.get_logger().info(
                f'Could not find transformation from base_link to lift_top: {ex}')
            return

        # Creating a transformation matrix out of the
        # translation and orientation values
        self.tf_matrix_base_link_to_lift_top = quaternion_matrix(
            [t.transform.rotation.x,
             t.transform.rotation.y,
             t.transform.rotation.z,
             t.transform.rotation.w])
        self.tf_matrix_base_link_to_lift_top[0][3] = t.transform.translation.x
        self.tf_matrix_base_link_to_lift_top[1][3] = t.transform.translation.y
        self.tf_matrix_base_link_to_lift_top[2][3] = t.transform.translation.z

        # Multiplying both the matrices to get the transformation
        # from odom to lift_top (forward kinematics)
        self.tf_matrix_odom_to_lift_top = np.matmul(
            self.tf_matrix_odom_to_base_link,
            self.tf_matrix_base_link_to_lift_top)

        # Obtaining values from the matrix
        self.quaternions = quaternion_from_matrix(
            self.tf_matrix_odom_to_lift_top)
        self.translation_x_odom_to_lift_top = \
            self.tf_matrix_odom_to_lift_top[0][3]
        self.translation_y_odom_to_lift_top = \
            self.tf_matrix_odom_to_lift_top[1][3]
        self.translation_z_odom_to_lift_top = \
            self.tf_matrix_odom_to_lift_top[2][3]

        self.get_logger().info(
            f'X : "{self.translation_x_odom_to_lift_top}" ,'
            f'Y : "{self.translation_y_odom_to_lift_top}" ,'
            f'Z : "{self.translation_z_odom_to_lift_top}"')

    def publisher_callback(self):
        odom = Odometry()

        odom.header.frame_id = "odom"
        odom.child_frame_id = "lift_top"
        odom.header.stamp = self.get_clock().now().to_msg()

        # set the position
        odom.pose.pose.position.x = self.translation_x_odom_to_lift_top
        odom.pose.pose.position.y = self.translation_y_odom_to_lift_top
        odom.pose.pose.position.z = self.translation_z_odom_to_lift_top
        odom.pose.pose.orientation.x = self.quaternions[0]
        odom.pose.pose.orientation.y = self.quaternions[1]
        odom.pose.pose.orientation.z = self.quaternions[2]
        odom.pose.pose.orientation.w = self.quaternions[3]

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    leica_pose_publisher = LeicaPosepublisher()

    rclpy.spin(leica_pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leica_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
