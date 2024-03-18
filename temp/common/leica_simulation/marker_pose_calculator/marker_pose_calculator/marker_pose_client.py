#  Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.

import sys

from bautiro_ros_interfaces.srv import LuControlTs16
import rclpy
from rclpy.node import Node


class MarkerPoseClientAsync(Node):

    def __init__(self):
        super().__init__('marker_pose_client_async')
        self.cli = self.create_client(LuControlTs16, 'get_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LuControlTs16.Request()

    def send_request(self, frame_id, x, y, z):
        self.req.in_point.header.frame_id = frame_id
        self.req.in_point.point.x = x
        self.req.in_point.point.y = y
        self.req.in_point.point.z = z
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    marker_pose_client = MarkerPoseClientAsync()
    response = marker_pose_client.send_request(
        str(sys.argv[1]), float(sys.argv[2]),
        float(sys.argv[3]), float(sys.argv[4]))
    if response.out_point.point.x == 0.0 and \
            response.out_point.point.y == 0.0 and \
            response.out_point.point.z == 0.0:
        marker_pose_client.get_logger().error(
            'Some error has occurred.'
            'Got x = 0.0, y = 0.0 and z = 0.0. Please check'
            ' the service terminal for detailed information.')
    else:
        marker_pose_client.get_logger().info(
            'Got: %s %s %s' % (
                response.out_point.point.x,
                response.out_point.point.y,
                response.out_point.point.z))

    marker_pose_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
