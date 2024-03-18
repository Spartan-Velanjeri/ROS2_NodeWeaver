#!/usr/bin/env python3
#
#  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
#
#  All rights reserved, also regarding any disposal, exploitation, reproduction,
#  editing, distribution, as well as in the event of applications for industrial
#  property rights.


from bautiro_ros_interfaces.srv import GetClusterPosition, GetClusterPose
from bautiro_ros_interfaces.msg import TransformationsMatrix
from geometry_msgs.msg import Vector3, Pose
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory


class GetClusterPositionService(Node):

    def __init__(self):
        super().__init__('get_cluster_server')
        self.srv = self.create_service(
            GetClusterPosition, 'get_cluster_position',
            self.get_cluster_position_callback)
        self.srv_pose = self.create_service(
            GetClusterPose, 'get_cluster_pose',
            self.get_cluster_pose_callback)
        # self.srv_num = self.create_service(
        #     GetNumPois, 'get_num_pois', self.get_num_pois_callback)
        self.pois = None
        self.num_pois = 0
        with open(get_package_share_directory(
                'p2p_offset_controller') + '/config/pois.yaml') as d:
            self.pois = yaml.load(d, yaml.loader.SafeLoader)
        # We have 4 entries per pose. If id or desription are removed,
        # divide by number of entries per pose
        self.num_pois = int(sum([len(x) for x in self.pois['pois']]) / 4)
    # def get_num_pois_callback(self, request, response):
    #     response.num_pois = self.num_pois
    #     self.get_logger().info(f'Sending POI Count {response.num_pois}')
    #     return response

    def get_cluster_pose_callback(self, request, response):
        id = request.number_of_next
        response.no_more_pois = False
        if id < 0:
            id = 0
        if id >= self.num_pois:
            self.get_logger().info('No more points available')
            response.no_more_pois = True
            return response
        requested_pose = self.pois['pois']['poi' + str(id)]['pose']
        pose = Pose()
        pose.position.x = requested_pose[0][0]
        pose.position.y = requested_pose[0][1]
        pose.position.z = requested_pose[0][2]
        pose.orientation.x = requested_pose[1][0]
        pose.orientation.y = requested_pose[1][1]
        pose.orientation.z = requested_pose[1][2]
        pose.orientation.w = requested_pose[1][3]
        self.get_logger().info(f'Sending Cluster Pose {id}: {requested_pose}')
        response.poi = pose
        return response

    def get_cluster_position_callback(self, request, response):
        id = request.number_of_next
        print(id)
        if id < 0:
            id = 0
        # requested_pose = self.pois['pois']['poi'+str(id)]['pose'][0]
        # self.get_logger().info(
        #     'Getting point of interest: #%d' % (request.number_of_next))
        if id > self.num_pois:
            id = self.num_pois
        pose_list = []
        for c in range(id):
            requested_pose = self.pois['pois']['poi' + str(c)]['pose']
            pose = Pose()
            pose.position.x = requested_pose[0][0]
            pose.position.y = requested_pose[0][1]
            pose.position.z = requested_pose[0][2]
            pose.orientation.x = requested_pose[1][0]
            pose.orientation.y = requested_pose[1][1]
            pose.orientation.z = requested_pose[1][2]
            pose.orientation.w = requested_pose[1][3]
            pose_list.append(pose)
        v_offest = Vector3()
        v_offest.x = 0.0
        v_offest.y = 0.0
        v_offest.z = 0.0
        v_x = Vector3()
        v_x.x = 1.0
        v_x.y = 0.0
        v_x.z = 0.0
        v_y = Vector3()
        v_y.x = 0.0
        v_y.y = 1.0
        v_y.z = 0.0
        v_z = Vector3()
        v_z.x = 0.0
        v_z.y = 0.0
        v_z.z = 1.0
        res = TransformationsMatrix()
        res.v0 = v_x
        res.v1 = v_y
        res.v2 = v_z
        res.v3 = v_offest
        response.cluster_list = [res]
        response.cluster_pose_list = pose_list
        return response


def main(args=None):
    rclpy.init(args=args)

    service = GetClusterPositionService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
