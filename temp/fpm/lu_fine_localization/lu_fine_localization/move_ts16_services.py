#!/usr/bin/env python3
#
# Copyright 2022 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa


import rclpy
from bautiro_ros_interfaces.srv import LuMeasureCeilingHeight, LuMoveToDustposition, LuTsMeasurePoint
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

class MoveTS16Services(Node):
    def __init__(self):
        super().__init__('move_ts16_services')
        # create and initialize subnodes and client for ts16 control service
        self.ts16_sub_node = rclpy.create_node('ts16_sub_node')
        self.control_ts16_sub_client = self.ts16_sub_node.create_client(LuTsMeasurePoint, '/fpm/sensors/ts16/lu_control_ts16') 
        # create movement services
        self.srv = self.create_service(LuMoveToDustposition, 'lu_move_to_dustposition', self.lu_move_to_dustposition)
        self.srv = self.create_service(LuMeasureCeilingHeight, 'lu_measure_ceiling_height', self.lu_measure_ceiling_height)

        self.get_logger().info('Moving TS16 Service is running ...')

    def lu_move_to_dustposition(self, request, response):
        service_name = request.NAME
        self.get_logger().info(f'Service is called: %s' % (service_name))
        # Move Leica TS16 to dust protection position (looking downwards to [0,0,-1])
        dustprotection_position = PointStamped()
        dustprotection_position.point.x = 0.0
        dustprotection_position.point.y = 0.0
        dustprotection_position.point.z = -1.0
        self.get_logger().info("Move Leica to dust protection position..........")
        # call control_ts16_sub_client
        while not self.control_ts16_sub_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service to control ts16 not available, waiting again...')
        dust_request = LuTsMeasurePoint.Request()
        dust_request.in_point = dustprotection_position
        future = self.control_ts16_sub_client.call_async(dust_request)
        rclpy.spin_until_future_complete(self.ts16_sub_node , future)
        if future.result() is not None:
            self.get_logger().info(f'Service called, result %s' % (future.result()))
            self.get_logger().info('TS16 moved to dust protection position.')
            response.success = True
            return response
        else:
            self.get_logger().error(f'Exception while calling service  %s: %s' % (service_name, future.exception()))

    def lu_measure_ceiling_height(self, request, response):
        service_name = request.NAME
        self.get_logger().info(f'Service is called: %s' % (service_name))
        # Move Leica TS16 to measure ceiling height (looking upwards to [0,0,1])
        ceiling_position = PointStamped()
        ceiling_position.point.x = 0.0
        ceiling_position.point.y = 0.0
        ceiling_position.point.z = 1.0
        self.get_logger().info("Measure ceiling height..........")

        # call control_ts16_sub_client
        while not self.control_ts16_sub_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service to control ts16 not available, waiting again...')
        ceiling_request = LuTsMeasurePoint.Request()
        ceiling_request.in_point = ceiling_position
        future = self.control_ts16_sub_client.call_async(ceiling_request)
        rclpy.spin_until_future_complete(self.ts16_sub_node , future)
        if future.result() is not None:
            result: LuTsMeasurePoint.Response = future.result()
            self.get_logger().info(f'Ceiling height measured: %s' % (result.out_point))
            response.out_point = result.out_point
            return response
        else:
            self.get_logger().error(f'Exception while calling service  %s: %s' % (service_name, future.exception()))


def main(args=None):
    rclpy.init(args=args)
    mts16s = MoveTS16Services()
    rclpy.spin(mts16s)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mts16s.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
