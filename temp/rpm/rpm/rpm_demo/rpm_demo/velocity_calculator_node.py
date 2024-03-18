#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator_node')
        # get ros paramters
        self.declare_parameter('input_topics', "/base_link/current_pose")
        self.declare_parameter('twist_output_topic', "current_velocity")
        self.declare_parameter('twist_frame_id', "map")

        self.input_topics = self.get_parameter('input_topics').value
        self.twist_frame_id = self.get_parameter('twist_frame_id').value
        self.twist_output_topic = self.get_parameter('twist_output_topic').value

        ###
        self.old_pose = PoseStamped()

        self.subPose = self.create_subscription(
            PoseStamped,
            self.input_topics,
            self.trans_vel_calc_callback,
            10)
        self.subPose  # prevent unused variable warning

        self.pubVel = self.create_publisher(TwistStamped, self.twist_output_topic, 10)

    def convert_rosPose_transform(self, ros_pose):

        quat = np.array(
            [
                ros_pose.orientation.x,
                ros_pose.orientation.y,
                ros_pose.orientation.z,
                ros_pose.orientation.w
            ]
        )
        pos = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
        rotMat = R.from_quat(quat)
        T = np.block([[rotMat.as_matrix(), pos.reshape(3, 1)], [0, 0, 0, 1]])
        return T

    def vel_calc(self, map_pose_map_robot_k, map_pose_map_robot_km1, delta_t):
        # convert Pose to transformation matrics and Rotation Matric
        # T_map_k = self.convert_rosPose_transform(map_pose_map_robot_k)
        T_map_km1 = self.convert_rosPose_transform(map_pose_map_robot_km1)
        R_map_km1 = T_map_km1[0:3, 0:3]

        # Robot position at k and km1 in map frame
        map_pos_map_robot_km1 = np.array(
            [
                map_pose_map_robot_km1.position.x,
                map_pose_map_robot_km1.position.y,
                map_pose_map_robot_km1.position.z
            ]
        )
        map_pos_map_robot_k = np.array(
            [
                map_pose_map_robot_k.position.x,
                map_pose_map_robot_k.position.y,
                map_pose_map_robot_k.position.z
            ]
        )

        map_pos_km1_k = map_pos_map_robot_k - map_pos_map_robot_km1
        km1_pos_km1_k = np.matmul(
            np.linalg.inv(R_map_km1),
            map_pos_km1_k.reshape(3, 1)
        )

        # velocity in body frame
        km1_vel_km1_k = km1_pos_km1_k / delta_t

        return km1_vel_km1_k

    def trans_vel_calc_callback(self, pose_msg):
        current_time = \
            pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec / 1000000000.0
        last_step_time = \
            self.old_pose.header.stamp.sec + self.old_pose.header.stamp.nanosec / 1000000000.0
        delta_t = current_time - last_step_time
        trans_vel = self.vel_calc(pose_msg.pose, self.old_pose.pose, delta_t)

        trans_vel_stamp = TwistStamped()
        trans_vel_stamp.header.frame_id = self.twist_frame_id
        trans_vel_stamp.header.stamp = pose_msg.header.stamp

        trans_vel_stamp.twist.linear.x = trans_vel[0, 0]
        trans_vel_stamp.twist.linear.y = trans_vel[1, 0]
        trans_vel_stamp.twist.linear.z = trans_vel[2, 0]

        trans_vel_stamp.twist.angular.x = 0.0
        trans_vel_stamp.twist.angular.y = 0.0
        trans_vel_stamp.twist.angular.z = 0.0

        self.pubVel.publish(trans_vel_stamp)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityCalculator()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    VelocityCalculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
