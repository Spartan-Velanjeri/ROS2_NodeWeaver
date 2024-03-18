#!/usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

import rclpy
from bautiro_ros_interfaces.srv import LuGetMarkerRoughPosition, \
    LuGetTargetPoseUrFrame, GetTrafo
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TransformServices(Node):
    def __init__(self):
        super().__init__('transform_services')

        # create triangulation service
        self.srv = self.create_service(
            LuGetMarkerRoughPosition,
            'lu_get_marker_rough_position',
            self.get_marker_rough_position_callback)
        self.srv = self.create_service(
            LuGetTargetPoseUrFrame,
            'lu_get_target_pose_ur_frame',
            self.get_target_pose_ur_frame_callback)
        self.srv = self.create_service(
            GetTrafo,
            'lu_get_trafo',
            self.get_trafo_general)

        # tf_listener initialisation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Running transform services ...')

    def get_marker_rough_position_callback(self, request, response):
        # convert input points to array
        in_markers = request.in_markers
        print(in_markers)
        marker_number = len(in_markers)

        # get rough position of markers from TF Tree
        print('Get rough position of the markers from TF Tree')
        leica_frame_id = 'fpm_totalstation'
        marker_rough_positions = []

        for i in range(marker_number):
            marker_id = in_markers[i].data

            # Look up for transform of the marker in leica frame
            try:
                # now = self.get_clock().now()
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    leica_frame_id,
                    marker_id,
                    now)  # , timeout=rclpy.duration.Duration(seconds=0.5))
                self.get_logger().info(
                    'Translation %s to %s: x=%f, y=%f, z=%f'
                    % (marker_id, leica_frame_id, trans.transform.translation.x,
                       trans.transform.translation.y, trans.transform.translation.z))
                self.get_logger().info(
                    'Orientation %s to %s: w=%f, x=%f, y=%f, z=%f'
                    % (marker_id, leica_frame_id, trans.transform.rotation.w,
                       trans.transform.rotation.x, trans.transform.rotation.y,
                       trans.transform.rotation.z))
            except TransformException as ex:
                self.get_logger().error(
                    f'Could not transform %s relative to %s: %s'
                    % (marker_id, leica_frame_id, ex))
                return False
            marker_rough_position = PointStamped()
            marker_rough_position.header.stamp = now.to_msg()
            marker_rough_position.header.frame_id = marker_id
            marker_rough_position.point.x = trans.transform.translation.x
            marker_rough_position.point.y = trans.transform.translation.y
            marker_rough_position.point.z = trans.transform.translation.z
            marker_rough_positions.append(marker_rough_position)

        # publish the calculated pose of leica in BIM coordinate
        self.get_logger().info('Calculated pose published')
        response.out_points = marker_rough_positions
        return response

    def get_target_pose_ur_frame_callback(self, request, response):
        # convert input points to array
        target_frame = request.target_frame
        ur_frame = request.ur_frame
        print(target_frame.data)

        # get rough position of markers from TF Tree
        print('Get target frame from TF Tree')

        # look up for transform
        try:
            now = self.get_clock().now()
            # now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                ur_frame.data,
                target_frame.data,
                now)  # , timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform %s relative to %s: %s' % (
                    target_frame.data, ur_frame.data, ex))
            return False
        target_pose_in_ur = PoseStamped()
        target_pose_in_ur.header.stamp = now.to_msg()
        target_pose_in_ur.header.frame_id = ur_frame.data
        target_pose_in_ur.pose.position.x = trans.transform.translation.x
        target_pose_in_ur.pose.position.y = trans.transform.translation.y
        target_pose_in_ur.pose.position.z = trans.transform.translation.z

        target_pose_in_ur.pose.orientation.x = trans.transform.rotation.x
        target_pose_in_ur.pose.orientation.y = trans.transform.rotation.y
        target_pose_in_ur.pose.orientation.z = trans.transform.rotation.z
        target_pose_in_ur.pose.orientation.w = trans.transform.rotation.w

        response.out_pose = target_pose_in_ur
        return response

    def get_trafo_general(self, request, response):
        # convert input points to array
        from_frame = request.from_frame
        to_frame = request.to_frame

        now = rclpy.time.Time()
        trafo = PoseStamped()
        trafo.header.stamp = now.to_msg()
        trafo.header.frame_id = to_frame.data

        # look up for transform
        try:
            trans = self.tf_buffer.lookup_transform(
                from_frame.data, to_frame.data, now)
            trafo.pose.position.x = trans.transform.translation.x
            trafo.pose.position.y = trans.transform.translation.y
            trafo.pose.position.z = trans.transform.translation.z
            trafo.pose.orientation.x = trans.transform.rotation.x
            trafo.pose.orientation.y = trans.transform.rotation.y
            trafo.pose.orientation.z = trans.transform.rotation.z
            trafo.pose.orientation.w = trans.transform.rotation.w
            self.get_logger().info(
                f'Send out transform %s to %s: (%f,%f,%f)' % (
                    from_frame.data, to_frame.data, trafo.pose.position.x,
                    trafo.pose.position.y, trafo.pose.position.z))
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform %s to %s: %s. Send out (0,0,0)' % (
                    from_frame.data, to_frame.data, ex))

        response.out_pose = trafo
        return response


def main(args=None):
    rclpy.init(args=args)
    ts = TransformServices()
    rclpy.spin(ts)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
