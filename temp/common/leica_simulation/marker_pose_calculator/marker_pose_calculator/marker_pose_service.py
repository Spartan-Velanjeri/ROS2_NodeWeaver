#  Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.

import rclpy
import sys
from rclpy.node import Node
from bautiro_ros_interfaces.srv import LuControlTs16
from tf_transformations import quaternion_matrix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
from numpy.linalg import inv
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import math


class MarkerPoseService(Node):

    def __init__(self):
        super().__init__('marker_pose_service')

        self.declare_parameter('reference_frame_param', 'leica')

        self.marker_flag = False
        self.reference_frame_flag = False
        self.traverser_visited = False
        self.frame_id_marker = None
        self.frame_id_reference = None
        self.default_reference_frame = None
        self.radius_limit = 100
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.mapping_coordinates_link_name = {}

        self.group = ReentrantCallbackGroup()

        self.marker_pose_wrt_reference_frame_srv = self.create_service(
            LuControlTs16, 'get_pose',
            self.calculate_marker_pose_wrt_reference_frame, callback_group=self.group)

        self.marker_publisher = self.create_publisher(
            String, 'marker_id', 10, callback_group=self.group)
        self.reference_frame_publisher = self.create_publisher(
            String, 'reference_frame', 10, callback_group=self.group)

        self.subscribe_marker_pose = self.create_subscription(
            PoseStamped,
            'marker_pose',
            self.marker_pose_callback,
            10, callback_group=self.group)
        self.subscribe_marker_pose  # prevent unused variable warning

        self.subscribe_reference_frame_pose = self.create_subscription(
            PoseStamped,
            'reference_frame_pose',
            self.reference_frame_pose_callback,
            10, callback_group=self.group)
        self.subscribe_reference_frame_pose
        # prevent unused variable warning

        self.publish_default_reference_frame()
        # Function call to publish leica as default reference frame

    def publish_default_reference_frame(self):
        msg = String()
        self.default_reference_frame = self.get_parameter('reference_frame_param') \
            .get_parameter_value().string_value
        msg.data = self.default_reference_frame
        self.reference_frame_publisher.publish(msg)

    def marker_pose_callback(self, msg):
        translation = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.frame_id_marker = msg.header.frame_id
        self.marker_pose_transformation_matrix = quaternion_matrix(quat)

        for index in range(3):
            self.marker_pose_transformation_matrix[index][3] = translation[index]

        self.marker_flag = True

    def reference_frame_pose_callback(self, msg):
        translation = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.frame_id_reference = msg.header.frame_id
        self.reference_frame_pose_transformation_matrix = quaternion_matrix(quat)

        for index in range(3):
            self.reference_frame_pose_transformation_matrix[index][3] = translation[index]

        self.reference_frame_flag = True

    def tf_tree_traverser(self):
        msg = String()
        frames_dict = yaml.safe_load(self._tf_buffer.all_frames_as_yaml())
        # Works only if called inside a callback, else empty list
        for marker_id in frames_dict.keys():
            if frames_dict[marker_id]['parent'] == 'markers':
                msg.data = marker_id
                time.sleep(0.1)
                while True:
                    self.marker_publisher.publish(msg)
                    if self.frame_id_marker == marker_id:
                        self.mapping_coordinates_link_name[marker_id] = [
                            self.marker_pose_transformation_matrix[0][3],
                            self.marker_pose_transformation_matrix[1][3],
                            self.marker_pose_transformation_matrix[2][3]]
                        break

    def calculate_marker_pose_wrt_reference_frame(self, request, response):

        msg = String()
        msg.data = request.in_point.header.frame_id

        if msg.data != self.frame_id_marker:
            if not self.traverser_visited:
                self.tf_tree_traverser()
                self.traverser_visited = True

            msg.data = ""
            coordinates = [
                request.in_point.point.x,
                request.in_point.point.y,
                request.in_point.point.z]
            min_difference = sys.maxsize
            for key, value in self.mapping_coordinates_link_name.items():
                sum_of_squares = (value[0] - coordinates[0])**2 + \
                    (value[1] - coordinates[1])**2 + \
                    (value[2] - coordinates[2])**2
                distance_from_marker = math.sqrt(sum_of_squares)

                if distance_from_marker <= self.radius_limit and \
                        distance_from_marker < min_difference:

                    min_difference = distance_from_marker
                    msg.data = key

            if msg.data == "":
                self.get_logger().warn('The given input is too far from all markers.'
                                       'Please try again.')
                return response

            print(msg.data)  # This print is just for debugging.
            # It'll tell us which marker it has chosen from the approximate values.
            self.marker_publisher.publish(msg)
            self.get_logger().info('Marker coordinates received. Sending pose of the marker.')
            time.sleep(0.1)

        if self.marker_flag and self.reference_frame_flag:
            self.marker_pose_wrt_reference_frame_matrix = \
                np.matmul(inv(self.reference_frame_pose_transformation_matrix),
                          self.marker_pose_transformation_matrix)
            if self.frame_id_marker == msg.data and \
                    self.frame_id_reference == self.default_reference_frame:
                response.out_point.point.x = \
                    self.marker_pose_wrt_reference_frame_matrix[0][3]
                response.out_point.point.y = \
                    self.marker_pose_wrt_reference_frame_matrix[1][3]
                response.out_point.point.z = \
                    self.marker_pose_wrt_reference_frame_matrix[2][3]
            else:
                self.get_logger().warn(
                    'The requested marker does not match with the actual data.'
                    'Please call the service again.')
        else:
            self.get_logger().warn(
                'Either marker pose or reference frame pose message is not received.'
                'Please check if the marker name is published to the topic:'
                '/marker_id and reference frame name is published to the topic : /reference_frame')
        return response


def main(args=None):
    rclpy.init(args=args)

    marker_pose_service = MarkerPoseService()

    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(marker_pose_service)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    executor.shutdown()
    marker_pose_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
