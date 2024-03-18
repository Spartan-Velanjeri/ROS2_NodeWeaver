#!/usr/bin/env python3
#
# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

import sys

import rclpy
import tf_transformations

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

import os
from ament_index_python.packages import get_package_share_directory



class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('tf_reflex_marker_broadcaster')

        # create timer callback 
        timer_period = 0.5 #second
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create publisher for marker
        self.target_figure_publisher = self.create_publisher(Marker, 'target_rviz_figure', 10)
        self.target_position_x = 0.0
        self.target_position_y = 0.0
        self.target_position_z = 0.0


        # # path for test local
        # fname_asc = '/home/ngq3fe/workspace/bautiro/colcon_ws/src/lu_fine_localization/config/Bosch_Le_131_Festpunkte_LeicaScan.asc'                
        # # path for wägeler
        # fname_asc = '/home/bautiro/driver_ws/src/lu_fine_localization/config/Bosch_Le_131_Festpunkte_LeicaScan.asc'           
        fname_asc = os.path.join(get_package_share_directory('lu_fine_localization'), 'config', 'Bosch_Le_131_Festpunkte_LeicaScan.asc')
        
        f = open(fname_asc,'r')
        lines = f.readlines()        
        marker_num = len(lines)
        self._tf_publisher_arr = [StaticTransformBroadcaster(self) for i in range(marker_num)]
        
        str_print = ''
        marker_idx = 0
        for line in lines:
            line_without_multiple_space = " ".join(line.split())
            marker_data = line_without_multiple_space.split()
            point_number_str = marker_data[0]
            rechtwert = float(marker_data[1])
            hochwert = float(marker_data[2])
            hoehe = float(marker_data[3])
            #[point_number, rechtwert, hochwert, hoehe] = [float(x) for x in marker_data]
            #point_number_str = str(int(point_number-1030000))
            #x_gk = rechtwert - gauss_krueger_zone*1000000 - false_easting - x_offset
            x_gk = rechtwert
            y_gk = hochwert
            z_gk = hoehe

            str_print += point_number_str 
            str_print += ","
            
            # Publish static transforms once at startup            
            self.make_transforms_quat(self._tf_publisher_arr[marker_idx],'map', point_number_str, x_gk, y_gk, z_gk, 0.0, 0.0, 0.0, 1.0)

            if (point_number_str=='target'):
                self.target_position_x = x_gk
                self.target_position_y = y_gk
                self.target_position_z = z_gk
            marker_idx = marker_idx + 1

        f.close()
        self.get_logger().info(str_print)
        self.get_logger().info('%s Markers published'%(str(marker_num)))

        self._tf_publisher = StaticTransformBroadcaster(self)
        self._tf_publisher1 = StaticTransformBroadcaster(self)
        self._tf_publisher2 = StaticTransformBroadcaster(self)
        
        # publish tf_transform from leica to ur16_base
        postfix_str = '_fein'
        prefix_str = ''

        # calibration data from leica TS60 to rbase for Waegele
        #pos_leica_rbase = [-0.006015915752723, -0.308399049510694, -0.352013013986282]
        #quat_leica_rbase = [-0.00395890753253281, -0.707793483486935, 0.706400126567287,0.003401906862437]

        # # calibration data from leica TS16 to rbase for FUS1 on 09.05.2023 (robotarm on side)
        # pos_leica_rbase = [-0.3004, 0.3616, -0.3331]
        # quat_leica_rbase = [-0.7080, -0.0179,0.0225, 0.7056] # xyzw       

        ## calibration data from leica TS16 to rbase for FUS1 on 20.06.2023 (robotarm on front)                
        #pos_leica_rbase = [-0.242719804569714, 0.192893476226860, -0.390714529219951]
        #quat_leica_rbase = [-0.446644802426233, 0.477797603466129,0.509305138038631, 0.559308633006059] # xyzw       

        # # calibration data from leica TS16 to rbase for FUS1 on 20.06.2023 (robotarm on front)                
        # pos_leica_rbase = [0.192893476226860, 0.242719804569714, -0.390714529219951]
        # quat_leica_rbase = [0.0220, 0.6537, -0.0354, 0.7556] # xyzw 

        # # calibration data from leica TS16 to rbase for FUS1 on 20.06.2023 (robotarm on front)                
        # pos_leica_rbase = [0.5136,-0.1312,-0.3285]
        # pos_leica_rbase = [0.5336,-0.2312,-0.3285]  # prism
        pos_leica_rbase = [0.4578,-0.2238,-0.3297]
        #quat_leica_rbase = [0.0527,0.6979,-0.0555,0.7121] # xyzw 
        # quat_leica_rbase = [-0.4562,0.5307,-0.5428,0.4643] # xyzw 
        quat_leica_rbase = [-0.4654,0.5327,-0.5334,0.4638] # xyzw 



        self.make_transforms_quat(self._tf_publisher,prefix_str+'leica'+postfix_str, prefix_str+'ur16_base'+postfix_str,
                            pos_leica_rbase[0],pos_leica_rbase[1],pos_leica_rbase[2], quat_leica_rbase[0], quat_leica_rbase[1], quat_leica_rbase[2], quat_leica_rbase[3])    
        
        self.make_transforms_quat(self._tf_publisher1,'leica', 'ur16_base_rough',
                            pos_leica_rbase[0],pos_leica_rbase[1],pos_leica_rbase[2], quat_leica_rbase[0], quat_leica_rbase[1], quat_leica_rbase[2], quat_leica_rbase[3])    
        
        # To be fix: add help transformation for planning in movit
        self.make_transforms_quat(self._tf_publisher2,prefix_str+'ur16_base'+postfix_str, 'ur16_base_movit_planning',
                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)

       
    def timer_callback(self):
        # create 
        target_rviz_marker = Marker()

        # set value for marker
        target_rviz_marker.header.frame_id = "map"
        target_rviz_marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        target_rviz_marker.type = 3
        target_rviz_marker.id = 0

        # Set the scale of the TARGET marker
        target_rviz_marker.scale.x = 0.25
        target_rviz_marker.scale.y = 0.25
        target_rviz_marker.scale.z = 0.2

        # Set the color of the TARGET marker
        target_rviz_marker.color.r = 1.0
        target_rviz_marker.color.g = 1.0
        target_rviz_marker.color.b = 1.0
        target_rviz_marker.color.a = 1.0

        # Set the pose of the marker
        target_rviz_marker.pose.position.x = self.target_position_x
        target_rviz_marker.pose.position.y = self.target_position_y
        target_rviz_marker.pose.position.z = self.target_position_z
        target_rviz_marker.pose.orientation.x = 0.0
        target_rviz_marker.pose.orientation.y = 0.0
        target_rviz_marker.pose.orientation.z = 0.0
        target_rviz_marker.pose.orientation.w = 1.0
        
        # publish marker
        self.target_figure_publisher.publish(target_rviz_marker)


    def make_transforms(self, tf_publisher, parent_frame_id, frame_id, x_trans, y_trans, z_trans, roll, pitch, yaw):
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

        tf_publisher.sendTransform(static_transformStamped)

    def make_transforms_quat(self,tf_publisher, parent_frame_id, frame_id, x_trans, y_trans, z_trans, quat_x, quat_y, quat_z, quat_w):
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

        tf_publisher.sendTransform(static_transformStamped)


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


if __name__ == '__main__':
    main()
