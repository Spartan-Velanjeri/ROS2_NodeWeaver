#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

# import for ros2
# import for function
from cmath import pi

import numpy as np
import rclpy
import tf_transformations as tf
from bautiro_ros_interfaces.msg import Marker
from bautiro_ros_interfaces.srv import (
    GetTrafo,
    LuGetMarkerRoughPosition,
    LuGetTargetPoseUrFrame,
    LuTsMeasurePoint,
)
from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from lu_fine_localization.pose_rigid_body import *
from lu_fine_localization.srv import LuFineLocalization
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

# import for transform listener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener


class FineLocalization(Node):
    def __init__(self):
        super().__init__("fine_localization")

        # Deadlock during call of a service inside of a service/callback (service in service)
        # can be fixed easily by assigning the callbacks to different callback groups
        # https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html
        main_cb_group = MutuallyExclusiveCallbackGroup()
        client_cb_group = MutuallyExclusiveCallbackGroup()

        # create fine_localization service
        self.srv = self.create_service(
            LuFineLocalization,
            LuFineLocalization.Request.NAME,
            self.fineloc_callback,
            callback_group=main_cb_group,
        )

        # create and initialize client for transformation service /lu_get_trafo
        trafo_client_str = "/lu_get_trafo"
        self.trafo_client = self.create_client(
            GetTrafo, trafo_client_str, callback_group=client_cb_group
        )
        while not self.trafo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service %s not available, waiting again..." % (trafo_client_str)
            )

        # create and initialize client for ts16 control service
        control_ts16_client_str = "/fpm/sensors/totalstation/lu_control_ts16"
        self.control_ts16_client = self.create_client(
            LuTsMeasurePoint, control_ts16_client_str, callback_group=client_cb_group
        )
        while not self.control_ts16_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service %s not available, waiting again..." % (control_ts16_client_str)
            )

        # publisher for visualisation
        self.pub_position = self.create_publisher(
            PoseWithCovarianceStamped, "/fine/ts16/pose", 10
        )

        # staic transform pusblisher
        self.tf_publisher = StaticTransformBroadcaster(self)

        self.get_logger().info(
            "Fine localization service /%s running ..."
            % (LuFineLocalization.Request.NAME)
        )

    def triangulation(self, mrks_measured: list, mrks_in_map: list):
        """
        Triangulation of the totalstation pose in map frame with at least 3 points
        INPUT:
            mrks_measured:  list of geometry_msgs.msg.PointStamped
            mrks_in_map:    list of bautiro_ros_interfaces.msg.Marker
        OUTPUT:
            PoseWithCovarianceStamped
        """

        if not isinstance(mrks_measured, list):
            raise TypeError
        if not isinstance(mrks_in_map, list):
            raise TypeError

        if not isinstance(mrks_measured[0], PointStamped):
            raise TypeError
        if not isinstance(mrks_in_map[0], Marker):
            raise TypeError

        if len(mrks_measured) < 3 or len(mrks_in_map) < 3:
            raise ValueError

        # Convert mrks_in_map into appropriate format #############################################
        mrks_in_map_ = []
        for mrk in mrks_in_map:
            mrk_ = PointStamped()
            mrk_.header = mrk.pose.header
            mrk_.point = mrk.pose.pose.position
            mrks_in_map_.append(mrk_)

        pose_ts_in_map = pose_rigid_body(mrks_measured, mrks_in_map_)

        return pose_ts_in_map

    def chain_trafos(self, frm1_to_frm2, frm2_to_frm3):
        """
        Combine transformations in a chain (e.g. TS --> MARKER from TS --> MAP and MAP --> MARKER)
        INPUT:
            frm1_to_frm2:  (geometry_msgs.msg.PoseStamped) transformation from frame1 to frame2
            frm2_to_frm3:  (geometry_msgs.msg.PoseStamped) transformation from frame2 to frame3

        OUTPUT:
            frm1_to_frm3:  (geometry_msgs.msg.PoseStamped) transformation from frame1 to frame 3
        """

        # Calculate the transformation
        tf_12 = tf.translation_matrix(
            [
                frm1_to_frm2.pose.position.x,
                frm1_to_frm2.pose.position.y,
                frm1_to_frm2.pose.position.z,
            ]
        ).dot(
            tf.quaternion_matrix(
                [
                    frm1_to_frm2.pose.orientation.x,
                    frm1_to_frm2.pose.orientation.y,
                    frm1_to_frm2.pose.orientation.z,
                    frm1_to_frm2.pose.orientation.w,
                ]
            )
        )
        tf_23 = tf.translation_matrix(
            [
                frm2_to_frm3.pose.position.x,
                frm2_to_frm3.pose.position.y,
                frm2_to_frm3.pose.position.z,
            ]
        ).dot(
            tf.quaternion_matrix(
                [
                    frm2_to_frm3.pose.orientation.x,
                    frm2_to_frm3.pose.orientation.y,
                    frm2_to_frm3.pose.orientation.z,
                    frm2_to_frm3.pose.orientation.w,
                ]
            )
        )
        tf_13 = tf_12.dot(tf_23)
        tf_13_position = tf.translation_from_matrix(tf_13)
        tf_13_orientation = tf.quaternion_from_matrix(tf_13)

        # Return the result
        frm1_to_frm3 = PoseStamped()
        frm1_to_frm3.header.stamp = frm1_to_frm2.header.stamp
        frm1_to_frm3.header.frame_id = frm1_to_frm2.header.frame_id
        frm1_to_frm3.pose.position = Point(
            x=tf_13_position[0], y=tf_13_position[1], z=tf_13_position[2]
        )
        frm1_to_frm3.pose.orientation = Quaternion(
            x=tf_13_orientation[0],
            y=tf_13_orientation[1],
            z=tf_13_orientation[2],
            w=tf_13_orientation[3],
        )
        return frm1_to_frm3

    def call_trafo(self, from_str, to_str):
        trafo_client_req = GetTrafo.Request()
        trafo_client_res = GetTrafo.Response()
        trafo_client_req.from_frame.data = from_str
        trafo_client_req.to_frame.data = to_str
        trafo_client_res = self.trafo_client.call_async(trafo_client_req)
        rclpy.spin_until_future_complete(self, trafo_client_res)
        return trafo_client_res.result().out_pose

    def call_ts_measure_point(self, mrk_rough):
        """
        INPUT:
            mrk_rough: geometry_msgs/PointStamped or geometry_msgs/PoseStamped
        OUTPUT:
            geometry_msgs/PointStamped
        """

        control_ts16_req = LuTsMeasurePoint.Request()
        control_ts16_res = LuTsMeasurePoint.Response()

        if isinstance(mrk_rough, PointStamped):
            control_ts16_req.in_point.header = mrk_rough.header
            control_ts16_req.in_point.point = mrk_rough.point
        elif isinstance(mrk_rough, PoseStamped):
            control_ts16_req.in_point.header = mrk_rough.header
            control_ts16_req.in_point.point = mrk_rough.pose.position
        else:
            raise TypeError

        control_ts16_res = self.control_ts16_client.call_async(control_ts16_req)
        rclpy.spin_until_future_complete(self, control_ts16_res)
        return control_ts16_res.result().out_point

    def fineloc_callback(self, request, response):
        self.get_logger().info(
            "Starting fine localization with %i markers ..." % (len(request.markers))
        )

        ts_frame_id = "fpm_totalstation"
        ts_fine_frame_id = "fpm_totalstation_fine"
        map_frame_id = "map"

        # Get rough pose of the totalstation (transformation: TS -> Map) ##########################
        tf_ts2map = self.call_trafo(ts_frame_id, map_frame_id)

        # Get rough pose of the markers in TS frame ###############################################
        mrks_rough = []
        for i in range(len(request.markers)):
            mrk = request.markers[i]
            mrks_rough.append(self.chain_trafos(tf_ts2map, mrk.pose))
            self.get_logger().info(
                "Transformed marker #%i to TS frame: (%.3f,%.3f,%.3f) --> (%.3f,%.3f,%.3f)."
                % (
                    i + 1,
                    mrk.pose.pose.position.x,
                    mrk.pose.pose.position.y,
                    mrk.pose.pose.position.z,
                    mrks_rough[-1].pose.position.x,
                    mrks_rough[-1].pose.position.y,
                    mrks_rough[-1].pose.position.z,
                )
            )

        # Call totalstation service to measure points #############################################
        mrks_fine = []
        for i in range(len(mrks_rough)):
            self.get_logger().info(
                "Calling totalstation service to measure the marker #%i (%.3f,%.3f,%.3f)"
                % (
                    i + 1,
                    mrks_rough[i].pose.position.x,
                    mrks_rough[i].pose.position.y,
                    mrks_rough[i].pose.position.z,
                )
            )
            ts_response = self.call_ts_measure_point(mrks_rough[i])
            self.get_logger().info(
                "Measurement result for marker #%i: %.3f,%.3f,%.3f"
                % (i + 1, ts_response.point.x, ts_response.point.y, ts_response.point.z)
            )
            mrks_fine.append(ts_response)

        #  Triangulation  #########################################################################
        tf_ts2map_fine = self.triangulation(mrks_fine, request.markers)
        response.pose = tf_ts2map_fine
        tf_ts2map_fine_rpy = tf.euler_from_quaternion(
            [
                tf_ts2map_fine.pose.pose.orientation.w,
                tf_ts2map_fine.pose.pose.orientation.x,
                tf_ts2map_fine.pose.pose.orientation.y,
                tf_ts2map_fine.pose.pose.orientation.z,
            ],
            axes="sxyz",
        )

        # Publish TS pose in map frame ############################################################
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = map_frame_id
        static_transformStamped.child_frame_id = ts_fine_frame_id
        static_transformStamped.transform.translation.x = (
            tf_ts2map_fine.pose.pose.position.x
        )
        static_transformStamped.transform.translation.y = (
            tf_ts2map_fine.pose.pose.position.y
        )
        static_transformStamped.transform.translation.z = (
            tf_ts2map_fine.pose.pose.position.z
        )
        static_transformStamped.transform.rotation.x = (
            tf_ts2map_fine.pose.pose.orientation.x
        )
        static_transformStamped.transform.rotation.y = (
            tf_ts2map_fine.pose.pose.orientation.y
        )
        static_transformStamped.transform.rotation.z = (
            tf_ts2map_fine.pose.pose.orientation.z
        )
        static_transformStamped.transform.rotation.w = (
            tf_ts2map_fine.pose.pose.orientation.w
        )
        self.tf_publisher.sendTransform(static_transformStamped)
        self.get_logger().info(
            "Published TF: %s -> %s." % (map_frame_id, ts_fine_frame_id)
        )

        # Move Leica to dust protection position (looking downwards to [0,0,-1]) ##################
        dustprotection_position = PointStamped()
        dustprotection_position.point.x = 0.0
        dustprotection_position.point.y = 0.0
        dustprotection_position.point.z = -1.0
        self.get_logger().info("Moving Leica to dust protection position ...")
        self.call_ts_measure_point(dustprotection_position)

        self.get_logger().info(
            "Return TS pose: xyz=(%.3f,%.3f,%.3f) rpy=(%.3f,%.3f,%.3f) xyzw=(%.3f,%.3f,%.3f,%.3f)"
            % (
                response.pose.pose.pose.position.x,
                response.pose.pose.pose.position.y,
                response.pose.pose.pose.position.z,
                tf_ts2map_fine_rpy[2],
                tf_ts2map_fine_rpy[1],
                tf_ts2map_fine_rpy[0],
                response.pose.pose.pose.orientation.x,
                response.pose.pose.pose.orientation.y,
                response.pose.pose.pose.orientation.z,
                response.pose.pose.pose.orientation.w,
            )
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    srv = FineLocalization()
    # executor = MultiThreadedExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(srv)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    srv.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
