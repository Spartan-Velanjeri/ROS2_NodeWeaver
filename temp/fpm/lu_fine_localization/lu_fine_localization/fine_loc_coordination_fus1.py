#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

# service command: ros2 service call /lu_fine_localization bautiro_ros_interfaces/srv/LuFineLocalization '{in_markers: [data: "1", data: "2", data: "3"]}'

import sys
import time
import subprocess
import os

import rclpy
from bautiro_ros_interfaces.srv import (LuTsMeasurePoint, LuFineLocalization,
                                        LuTriangulation, LuGetMarkerRoughPosition,LuGetTargetPoseUrFrame, GetTrafo,
                                        LuMeasureCeilingHeight, LuMoveToDustposition,)
from bautiro_ros_interfaces.action import SetHandlingUnitConfiguredPose, MoveHandlingUnitAbsolute, MoveLiftAbsolute

from lu_waegele.fine_publish_pivot import GetGroundTrouth


from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock
from std_msgs.msg import String
# import for transform listener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped,PoseStamped
import tf_transformations

from scipy.spatial.transform import Rotation


class FeinLocCoordination(Node):
    def __init__(self):
        super().__init__('fein_loc_coordination')
        # # create triangulation service
        # self.fein_loc_coordination_server = self.create_service(LuFineLocalization, 'lu_fine_localization',
        #                                 self.lu_fine_localization_callback)

        # tf_listener initialisation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create and initialize action client for SetHandlingUnitConfiguredPose
        self._ac_configured_pose = ActionClient(self, SetHandlingUnitConfiguredPose, '/fpm_set_hu_configured_pose')

        # create and initialize action client for SetHandlingUnitConfiguredPose
        self._lift_configured_pose = ActionClient(self, MoveLiftAbsolute, '/move_lift_absolute')

        # create and initialize action client for MoveHandlingUnitAbsolute
        self._ac_absolute_pose = ActionClient(self, MoveHandlingUnitAbsolute, '/fpm_set_move_hu_absolute')

        # create and initialize client for get_target_pose_ur_frame service
        self.get_target_pose_ur_frame_client = self.create_client(LuGetTargetPoseUrFrame, '/lu_get_target_pose_ur_frame')
        while not self.get_target_pose_ur_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get_Target_Pose_Ur_Frame not available, waiting again...')
        self.get_target_pose_req = LuGetTargetPoseUrFrame.Request()
        
        
        # create and initialize client for transformation service /lu_get_trafo
        self.trafo_client = self.create_client(GetTrafo, '/lu_get_trafo')
        while not self.trafo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service LU_GET_TRAFO_GENERAL not available, waiting again...')
        self.trafo_client_req = GetTrafo.Request()


        # create and initialize client for get_marker_rough_position service
        self.get_rough_positon_client = self.create_client(LuGetMarkerRoughPosition, '/lu_get_marker_rough_position')
        while not self.get_rough_positon_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get_Marker_Rough_Position not available, waiting again...')
        self.get_rough_positon_req = LuGetMarkerRoughPosition.Request()


        # create and initialize client for triangulation service
        self.triangulation_client = self.create_client(LuTriangulation, '/lu_triangulation')
        while not self.triangulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Triangulation not available, waiting again...')
        self.triangulation_req = LuTriangulation.Request()

        # create and initialize client for ts16 control service
        self.control_ts16_client = self.create_client(LuTsMeasurePoint, '/fine/ts16/lu_control_ts16')
        while not self.control_ts16_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service to control ts16 not available, waiting again...')
        self.control_ts16_req = LuTsMeasurePoint.Request()

        # create and initialize client to move ts16 to dust protection position
        self.move_to_dustposition_client = self.create_client(LuMoveToDustposition, '/lu_move_to_dustposition')
        while not self.move_to_dustposition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service to move ts16 to dust protection position not available, waiting again...')
        self.move_to_dustposition_req = LuMoveToDustposition.Request()
        
        self.measure_ceiling_height_client = self.create_client(LuMeasureCeilingHeight, '/lu_measure_ceiling_height')
        while not self.measure_ceiling_height_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service to measure ceiling height with ts16 not available, waiting again...')
        self.measure_ceiling_height_req = LuMeasureCeilingHeight.Request()

        # publisher for visualisation
        self.pub_position = self.create_publisher(PoseWithCovarianceStamped, '/fine/ts16/pose', 10)

        # publisher for base_linc_fine to map 
        self.pub_base_link_fine = self.create_publisher(PoseWithCovarianceStamped, 'base_link_fine', 10)

        # publisher
        self.pub_pose_pivot = self.create_publisher(PoseStamped, '/fine/fine_pivot', 10)

        # tf publisher for tranform of leica in pks
        self._tf_publisher = StaticTransformBroadcaster(self)
        self._tf_publisher1 = StaticTransformBroadcaster(self)


    def set_arm_configured_pose_send_goal(self, configured_pose):    
        goal_msg = SetHandlingUnitConfiguredPose.Goal()
        goal_msg.handling_unit_configured_pose = configured_pose

        self.get_logger().info('Waiting for SetHandlingUnitConfiguredPose server...')
        self._ac_configured_pose.wait_for_server()

        self.get_logger().info('Send goal to SetHandlingUnitConfiguredPose server...')
        self.ac_future = self._ac_configured_pose.send_goal_async(goal_msg)
        time.sleep(5.0)
        rclpy.spin_until_future_complete(self, self.ac_future)
        self.get_logger().info('SetHandlingUnitConfiguredPose done.')

        return self.ac_future.result()


    def set_lift_configured_pose_send_goal(self, configured_pose):    
        goal_msg = MoveLiftAbsolute.Goal()
        goal_msg.requested_target_lift_level = configured_pose

        self.get_logger().info('Waiting for MoveLiftAbsolute server...')
        self._lift_configured_pose.wait_for_server()

        self.get_logger().info('Send goal to MoveLiftAbsolute server...')
        self.ac_future_lift = self._lift_configured_pose.send_goal_async(goal_msg)
        time.sleep(20.0)
        rclpy.spin_until_future_complete(self, self.ac_future_lift)
        self.get_logger().info('MoveLiftAbsolute done.')

        return self.ac_future_lift.result()


    def move_arm_absolute_pose_send_goal(self, absolute_position):    
        goal_msg = MoveHandlingUnitAbsolute.Goal()
        goal_msg.absolute_target_position = absolute_position

        self.get_logger().info('Waiting for MoveHandlingUnitAbsolute server...')
        self._ac_absolute_pose.wait_for_server()

        self.get_logger().info('Send goal to MoveHandlingUnitAbsolute server...')
        self.ac_future = self._ac_absolute_pose.send_goal_async(goal_msg)
        time.sleep(5.0)
        rclpy.spin_until_future_complete(self, self.ac_future)
        self.get_logger().info('MoveHandlingUnitAbsolute done...')

        return self.ac_future.result()


    def get_target_pose_send_request(self, target_frame, ur_frame):
        self.get_target_pose_req.target_frame = target_frame
        self.get_target_pose_req.ur_frame = ur_frame
        
        self.future = self.get_target_pose_ur_frame_client.call_async(self.get_target_pose_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('GetTargetPoseUrFrame service done...')
        return self.future.result()

    def get_rough_position_send_request(self, marker_ids):
        self.get_rough_positon_req.in_markers = marker_ids

        self.future = self.get_rough_positon_client.call_async(self.get_rough_positon_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('GetMarkerRoughPosition service done...')
        return self.future.result()

    def triangulation_send_request(self, marker_rough_postitions):
        self.triangulation_req.in_points = marker_rough_postitions
            #self.result = self.triangulation_client.call(self.triangulation_req)
        self.future = self.triangulation_client.call_async(self.triangulation_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Triangulation service done...')
        return self.future.result()
        #return self.result


    def control_ts16_send_request(self, marker_rough_position):
        self.control_ts16_req.in_point = marker_rough_position
        #self.result = self.triangulation_client.call(self.triangulation_req)
        #self.get_logger().info('Triangulation service done...')
        self.future = self.control_ts16_client.call_async(self.control_ts16_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('TS16 control service (measure one point) done.')
        return self.future.result()
        #return self.result


    def publish_trafos_for_gt_prisms(self):

        # Calibration data
        self.get_logger().info('Using Leica <-> Prisms calibration 20.04.2023! Please check!')
        miniprism_left_fine_TS16xyz =[-1.1929, 0.0642,-0.9473]
        miniprism_right_fine_TS16xyz=[-1.2018,-0.5923,-0.8073]
        miniprism_rear_fine_TS16xyz =[-1.6771,-0.6111,-0.5901]

        miniprism_left_fine_TS16xyzw =[0.0,0.0,0.0,1.0]
        miniprism_right_fine_TS16xyzw=[0.0,0.0,0.0,1.0]
        miniprism_rear_fine_TS16xyzw =[0.0,0.0,0.0,1.0]

        # Publish transform
        static_trafo_left = TransformStamped()
        static_trafo_left.header.stamp = self.get_clock().now().to_msg()
        static_trafo_left.header.frame_id = "leica_fein"
        static_trafo_left.child_frame_id = "miniprism_left_fine"
        static_trafo_left.transform.translation.x = miniprism_left_fine_TS16xyz[0]
        static_trafo_left.transform.translation.y = miniprism_left_fine_TS16xyz[1]
        static_trafo_left.transform.translation.z = miniprism_left_fine_TS16xyz[2]
        static_trafo_left.transform.rotation.x = miniprism_left_fine_TS16xyzw[0]
        static_trafo_left.transform.rotation.y = miniprism_left_fine_TS16xyzw[1]
        static_trafo_left.transform.rotation.z = miniprism_left_fine_TS16xyzw[2]
        static_trafo_left.transform.rotation.w = miniprism_left_fine_TS16xyzw[3]
        self.get_logger().info("Publishing TF leica_fein -> miniprism_left_fine ...")
        self._tf_publisher.sendTransform(static_trafo_left)

        static_trafo_right = TransformStamped()
        static_trafo_right.header.stamp = self.get_clock().now().to_msg()
        static_trafo_right.header.frame_id = "leica_fein"
        static_trafo_right.child_frame_id = "miniprism_right_fine"
        static_trafo_right.transform.translation.x = miniprism_right_fine_TS16xyz[0]
        static_trafo_right.transform.translation.y = miniprism_right_fine_TS16xyz[1]
        static_trafo_right.transform.translation.z = miniprism_right_fine_TS16xyz[2]
        static_trafo_right.transform.rotation.x = miniprism_right_fine_TS16xyzw[0]
        static_trafo_right.transform.rotation.y = miniprism_right_fine_TS16xyzw[1]
        static_trafo_right.transform.rotation.z = miniprism_right_fine_TS16xyzw[2]
        static_trafo_right.transform.rotation.w = miniprism_right_fine_TS16xyzw[3]
        self.get_logger().info("Publishing TF leica_fein -> miniprism_right_fine ...")
        self._tf_publisher.sendTransform(static_trafo_right)

        static_trafo_rear = TransformStamped()
        static_trafo_rear.header.stamp = self.get_clock().now().to_msg()
        static_trafo_rear.header.frame_id = "leica_fein"
        static_trafo_rear.child_frame_id = "miniprism_rear_fine"
        static_trafo_rear.transform.translation.x = miniprism_rear_fine_TS16xyz[0]
        static_trafo_rear.transform.translation.y = miniprism_rear_fine_TS16xyz[1]
        static_trafo_rear.transform.translation.z = miniprism_rear_fine_TS16xyz[2]
        static_trafo_rear.transform.rotation.x = miniprism_rear_fine_TS16xyzw[0]
        static_trafo_rear.transform.rotation.y = miniprism_rear_fine_TS16xyzw[1]
        static_trafo_rear.transform.rotation.z = miniprism_rear_fine_TS16xyzw[2]
        static_trafo_rear.transform.rotation.w = miniprism_rear_fine_TS16xyzw[3]
        self.get_logger().info("Publishing TF leica_fein -> miniprism_rear_fine ...")
        self._tf_publisher.sendTransform(static_trafo_rear)        
        
        # Publish topic

        trafo_left = self.call_trafo('map','miniprism_left_fine')
        self.orientation_for_pivot(trafo_left,trafo_left)
        
        # os.system("ros2 run lu_waegele fine_publish_pivot")


    def orientation_for_pivot(self,trafo_left,trafo_leica):
        # Generate gt_pivot msg
        pivot_pose_pks = PoseStamped()
        pivot_pose_pks.header.stamp = trafo_left.header.stamp
        pivot_pose_pks.header.frame_id = 'map'
        pivot_pose_pks.pose.position = trafo_left.pose.position
        pivot_pose_pks.pose.orientation = trafo_leica.pose.orientation
        # pivot_pose_pks.pose.orientation.w = trafo_leica.pose.orientation.w
        # pivot_pose_pks.pose.orientation.x = trafo_leica.pose.orientation.x
        # pivot_pose_pks.pose.orientation.y = trafo_leica.pose.orientation.y
        # pivot_pose_pks.pose.orientation.z = trafo_leica.pose.orientation.z
        self.pub_pose_pivot.publish(pivot_pose_pks)

        quat = [pivot_pose_pks.pose.orientation.x,
                pivot_pose_pks.pose.orientation.y,
                pivot_pose_pks.pose.orientation.z,
                pivot_pose_pks.pose.orientation.w]
        euler = Rotation.from_quat(quat).as_euler("xyz")

        # self.get_logger().info('Translation (fine): x=%.4f,  y=%.4f,  z=%.4f'% (pivot_pose_pks.pose.position.x, pivot_pose_pks.pose.position.y, pivot_pose_pks.pose.position.z))
        # self.get_logger().info('Orientation (fine): x=%.4f,  y=%.4f,  z=%.4f,  w=%.4f'% (pivot_pose_pks.pose.orientation.x, pivot_pose_pks.pose.orientation.y, pivot_pose_pks.pose.orientation.z, pivot_pose_pks.pose.orientation.w))
        # self.get_logger().info('Orientation (fine): r=%.2f°, p=%.2f°, y=%.2f°'%(euler[0]/3.1415*180.0,euler[1]/3.1415*180.0,euler[2]/3.1415*180.0))



    def call_trafo(self,from_str,to_str):
        self.trafo_client_req.from_frame.data = from_str
        self.trafo_client_req.to_frame.data = to_str
        self.resp = self.trafo_client.call_async(self.trafo_client_req)
        rclpy.spin_until_future_complete(self, self.resp)
        return self.resp.result().out_pose


    def run(self, in_markers):

        ########## move the lift
        self.get_logger().info('Move the lift to configured position')
        input("Press Enter to move the lift ...")
        self.set_lift_configured_pose_send_goal(1.37)

        ########## move robot arm to transport position
        self.get_logger().info('Move robot arm to configured position 4 (Measurement position)')
        
        # control robot manually
        #input("Press Enter to move robot...")
        
        # # control robot to position 4 by sh-script
        # print(subprocess.run(["/home/bautiro/driver_ws/src/lu_fine_localization/script/robot_goto_0.sh",""],shell=True))
        # # time.sleep(10.0)

        # # control robot using action 
        self.set_arm_configured_pose_send_goal(4)
        time.sleep(6.0) # wait 6s in addition for free view of ts16

        ########### Get Rough Position of the markers
        self.get_logger().info('Get rough position of the markers, call get rough position service')
        marker_rough_positions = []
        get_marker_rough_position_response = self.get_rough_position_send_request(in_markers)
        marker_rough_positions = get_marker_rough_position_response.out_points
        print(marker_rough_positions)
        
        ########### Control TS16 to the markers
        self.get_logger().info('Control TS16 to the markers, call control service')
        marker_fein_positions = []
        marker_number = len(marker_rough_positions)
        for i in range(marker_number):
            # service call to control ts16
            ts16_response = self.control_ts16_send_request(marker_rough_positions[i])
            marker_fein_positions.append(ts16_response.out_point)
            print("MEASUREMENT ===================")
            print(i)
            print(ts16_response.out_point)
            print("===============================")
            

        ########### Call triangulation service
        self.get_logger().info('Call triangulation service to calculate TS16 pose')
        triangulation_response = self.triangulation_send_request(marker_fein_positions)

        #response.leica_pose_pks = triangulation_response.out_pose
        leica_pose_pks = triangulation_response.out_pose

        self.get_logger().info('Translation: x=%f, y=%f, z=%f'
                    % (leica_pose_pks.pose.pose.position.x, leica_pose_pks.pose.pose.position.y, leica_pose_pks.pose.pose.position.z))


        # # publish the result
        # msg_pose = PoseWithCovarianceStamped()
        # msg_pose.header.frame_id  = "map" 
        # msg_pose.header.stamp     = Clock().now().to_msg()
        # msg_pose.pose.pose = leica_pose_pks.pose.pose
        # msg_pose.pose.covariance = leica_pose_pks.pose.covariance
        # # msg_pose.point.y = leica_pose_pks.pose.pose.position.y
        # # msg_pose.point.z = leica_pose_pks.pose.pose.position.z
        # # msg_pose.point. = leica_pose_pks.pose.pose
        # self.pub_position.publish(msg_pose)


        # # test with fake leica_pose_pks
        # leica_pose_pks = PoseWithCovarianceStamped()
        # leica_pose_pks.pose.pose.position.x = -9.0
        # leica_pose_pks.pose.pose.position.y = 3.0
        # leica_pose_pks.pose.pose.position.z = 2.0
        # leica_pose_pks.pose.pose.orientation.x=0.0
        # leica_pose_pks.pose.pose.orientation.y=0.0
        # leica_pose_pks.pose.pose.orientation.z=0.0
        # leica_pose_pks.pose.pose.orientation.w=1.0

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "leica_fein"
        static_transformStamped.transform.translation.x = leica_pose_pks.pose.pose.position.x
        static_transformStamped.transform.translation.y = leica_pose_pks.pose.pose.position.y
        static_transformStamped.transform.translation.z = leica_pose_pks.pose.pose.position.z
        static_transformStamped.transform.rotation.x = leica_pose_pks.pose.pose.orientation.x
        static_transformStamped.transform.rotation.y = leica_pose_pks.pose.pose.orientation.y
        static_transformStamped.transform.rotation.z = leica_pose_pks.pose.pose.orientation.z
        static_transformStamped.transform.rotation.w = leica_pose_pks.pose.pose.orientation.w
        self.get_logger().info("Publishing TF map -> leica_fein ...")
        self._tf_publisher.sendTransform(static_transformStamped)

        # # Publish trafos for ground truth prisms
        # self.publish_trafos_for_gt_prisms()

        # ############ Get transform leica_fein to base_link_fine
        # get_trafo_client_req = GetTrafo.Request()
        # frame_id_to = String()
        # frame_id_to.data = 'base_link_rough'
        # frame_id_from = String()
        # frame_id_from.data = 'leica'
        # get_trafo_client_req.from_frame = frame_id_from
        # get_trafo_client_req.to_frame = frame_id_to
        # self.leica_future = self.get_trafo_client.call_async(get_trafo_client_req)
        # rclpy.spin_until_future_complete(self, self.leica_future)
        # trafo_leica_fein_base_link_rough = self.leica_future.result().out_pose

        # static_transformStamped_leica = TransformStamped()
        # static_transformStamped_leica.header.stamp = self.get_clock().now().to_msg()
        # static_transformStamped_leica.header.frame_id = "leica_fein"
        # static_transformStamped_leica.child_frame_id = "base_link_fine"
        # static_transformStamped_leica.transform.translation.x = trafo_leica_fein_base_link_rough.pose.position.x
        # static_transformStamped_leica.transform.translation.y = trafo_leica_fein_base_link_rough.pose.position.y
        # static_transformStamped_leica.transform.translation.z = trafo_leica_fein_base_link_rough.pose.position.z
        # static_transformStamped_leica.transform.rotation.x = trafo_leica_fein_base_link_rough.pose.orientation.x
        # static_transformStamped_leica.transform.rotation.y = trafo_leica_fein_base_link_rough.pose.orientation.y
        # static_transformStamped_leica.transform.rotation.z = trafo_leica_fein_base_link_rough.pose.orientation.z
        # static_transformStamped_leica.transform.rotation.w = trafo_leica_fein_base_link_rough.pose.orientation.w

        # self.get_logger().info("Publishing TF leica_fein --> base_link_fine ...")
        # self._tf_publisher.sendTransform(static_transformStamped_leica)

        # ############ Get transform leica_fein to map
        # get_trafo_client_req_map = GetTrafo.Request()
        # frame_id_to_map = String()
        # frame_id_to_map.data = 'map'
        # frame_id_from_fine = String()
        # frame_id_from_fine.data = 'base_link_fine'
        # get_trafo_client_req_map.from_frame = frame_id_from_fine
        # get_trafo_client_req_map.to_frame = frame_id_to_map
        # self.leica_future_map = self.get_trafo_client.call_async(get_trafo_client_req_map)
        # rclpy.spin_until_future_complete(self, self.leica_future_map)
        # trafo_base_link_fine_map = self.leica_future_map.result().out_pose
        
        # # publish the result of base_link_fine in map
        # msg_base_link_fine = PoseWithCovarianceStamped()
        # msg_base_link_fine.header.frame_id  = "map" 
        # msg_base_link_fine.header.stamp     = Clock().now().to_msg()
        # msg_base_link_fine.pose.pose = trafo_base_link_fine_map.pose
        # self.pub_base_link_fine.publish(msg_base_link_fine)

        # Move TS16 to measure ceiling height at [0,0,1] 
        self.get_logger().info("Measure ceiling height..........")
        height = self.measure_ceiling_height_req()
        self.get_logger().info('Measured ceiling height: ', height.out_point)
        
        ############ Move Leica to dust protection position (looking downwards to [0,0,-1])
        # dustprotection_position = PointStamped()
        # dustprotection_position.point.x = 0.0
        # dustprotection_position.point.y = 0.0
        # dustprotection_position.point.z = -1.0
        self.get_logger().info("Move Leica to dust protection position..........")
        # self.control_ts16_send_request(dustprotection_position)
        moved = self.move_to_dustposition_req()
        if moved.success:
            self.get_logger().info("Move Leica to dust protection position was successful")

        ############ Get target pose in ur16_base
        self.get_logger().info("Get target pose in ur_base from TF..........")     
        # call service for Target Pose
        self.get_logger().info('Get target pose in ur_frame, call get_target_pose service')
        target_frame_str = String()
        target_frame_str.data = 'target'
        ur_frame_str = String()
        #ur_frame_str.data = 'ur16_base_fein'
        ur_frame_str.data = 'ur16_base_movit_planning'        
        get_target_pose_ur_frame_response = self.get_target_pose_send_request(target_frame_str,ur_frame_str)
        target_pose_in_ur = get_target_pose_ur_frame_response.out_pose

        # ########### To be fix: Change sign of x and y to fit with current frame
        # target_pose_in_ur.pose.position.x = -target_pose_in_ur.pose.position.x
        # target_pose_in_ur.pose.position.y = -target_pose_in_ur.pose.position.y
        # target_pose_in_ur.pose.position.z = target_pose_in_ur.pose.position.z        

        static_transformStamped1 = TransformStamped()
        static_transformStamped1.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped1.header.frame_id = "base_link"
        static_transformStamped1.child_frame_id = "desired_tcp"
        static_transformStamped1.transform.translation.x = target_pose_in_ur.pose.position.x
        static_transformStamped1.transform.translation.y = target_pose_in_ur.pose.position.y
        static_transformStamped1.transform.translation.z = target_pose_in_ur.pose.position.z
        static_transformStamped1.transform.rotation.x = target_pose_in_ur.pose.orientation.x
        static_transformStamped1.transform.rotation.y = target_pose_in_ur.pose.orientation.y
        static_transformStamped1.transform.rotation.z = target_pose_in_ur.pose.orientation.z
        static_transformStamped1.transform.rotation.w = target_pose_in_ur.pose.orientation.w
        self.get_logger().info("Publishing TF ur16_base_movit_planning -> desired_tcp ...")
        self._tf_publisher1.sendTransform(static_transformStamped1)


        ############# move robot to absolute position
        self.get_logger().info("Move robot to target position")
        self.get_logger().info('Translation x=%f, y=%f, z=%f' % (target_pose_in_ur.pose.position.x, target_pose_in_ur.pose.position.y, target_pose_in_ur.pose.position.z))
        self.get_logger().info('Orientation w_x=%f, w_y=%f, w_z=%f, w_w=%f' % (target_pose_in_ur.pose.orientation.x, target_pose_in_ur.pose.orientation.y, target_pose_in_ur.pose.orientation.z,target_pose_in_ur.pose.orientation.w))
        
        self.set_arm_configured_pose_send_goal(8)


        # move robot by action
        self.move_arm_absolute_pose_send_goal(target_pose_in_ur)


        self.get_logger().info('Moving robot arm to start position ...')
        input("Press Enter to continue...")
        self.set_arm_configured_pose_send_goal(8)

        print(target_pose_in_ur)

        return leica_pose_pks

def main(args=None):
    print('Hi from fein_loc_coordination.')
    rclpy.init(args=args)
    fein_loc_coordination_ins = FeinLocCoordination()

    lookup_succes = True
    while (lookup_succes):
        time.sleep(0.5)
        # list of markers to be measure
        marker_frame_ids = ['1310002','1310009','1310005']
        #marker_frame_ids = ['1310003','1310010','1310005']
        #marker_frame_ids = ['1310001','1310010','1310005']
        # hintere: 03g 10g 09g (10k->10g)
        # links: 04k 08g 07k 05g
        # vorn: 06k 01g
        # rechts: 02g 11k

        # marker_frame_ids = ['1310001','1310003','1310008'] # 120° ggg pos1,2,3
        # marker_frame_ids = ['1310001','1310009','1310008'] # 120° ggg pos5,6
        # marker_frame_ids = ['1310005','1310001','1310002'] # 90° ggg pos1,2,3
        # marker_frame_ids = ['1310011','1310009','1310008'] # 90° ggg pos4
        # marker_frame_ids = ['1310011','1310009','1310008'] # 90° ggg pos5,6
        # marker_frame_ids = ['1310097','1310098','1310099'] # 120° ppp pos1,2,3,5,6
        # marker_frame_ids = ['1310001','1310002','1310003','1310009','1310008','1310005'] # 360° Verteilung pos1,2,3
        # marker_frame_ids = ['1310001','1310002','1310011','1310009','1310008','1310006'] # 360° Verteilung pos5,6

        # marker_frame_ids = ['1310003','1310010','1310009'] # gkg eine kleine Wand
        # marker_frame_ids = ['1310001','1310003','1310008'] # ~120° ggg
        # marker_frame_ids = ['1310002','1310009','1310008'] # ~90° ggg
        # marker_frame_ids = ['1310004','1310011','1310010'] # ~90° kkk
        # marker_frame_ids = ['1310010','1310007','1310006'] # ~120° kkk
        # marker_frame_ids = ['1310003','1310010','1310009'] # ~0° ggg
        # marker_frame_ids = ['1310002','1310010','1310008'] # ~90° ggg
        # marker_frame_ids = ['1310001','1310009','1310008'] # ~90° ggg
        # marker_frame_ids = ['1310001','1310098','1310099'] # ~120°gpp


        # generate a list of markers
        in_markers = []
        for marker_id in marker_frame_ids:
            temp_marker = String()
            temp_marker.data = marker_id
            in_markers.append(temp_marker)           

        # Execute fein localization with markers list
        fein_loc_result = fein_loc_coordination_ins.run(in_markers)
        if (fein_loc_result==False):
            rclpy.spin_once(fein_loc_coordination_ins)
        else:
            return
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fein_loc_coordination_ins.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
