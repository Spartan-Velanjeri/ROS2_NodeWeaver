
import sys
import os
from time import sleep

import rclpy 
from rclpy.node import Node
from rclpy.clock import Clock
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
# from std_msgs.msg import Int64

import numpy as np
import open3d as o3d

class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_for_rokit_node')

        assert len(sys.argv) > 1, "No pts file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        pts_path = sys.argv[1]

        # Convert with Open3D in numpy array
        pcd = o3d.io.read_point_cloud(pts_path)
        self.points = np.asarray(pcd.points)
        self.get_logger().info('Read PTS file with following dimension:')
        print(self.points.shape)

        # Publisher        
        self.version_publisher = self.create_publisher(std_msgs.Int64, 'version', 10)
        self.pose_publisher    = self.create_publisher(geometry_msgs.PoseStamped, 'pose', 10)
        self.pcd_publisher     = self.create_publisher(sensor_msgs.PointCloud2, 'scan', 10)
        self.ground_scan_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'ground_scan', 10)

        self.map_version = 3
        self.frame_id_pose = 'map'
        self.frame_id_scan = 'os_sensor'
        self.nr_of_steps = 100                # [count]
        self.step_size = 0.1                  # [meter]
        self.publish_timeinterval = 0.1       # [seconds]

        # Publish twice
        sleep(3.0)
        self.run()
        sleep(1.0)


    def run(self):
        """run"""
        for i in range(self.nr_of_steps):
            offset = np.array([float(i*self.step_size),0.0,0.0])
            points_new = self.add_offset(self.points,offset)
            self.publish(points_new,offset)
            sleep(self.publish_timeinterval)
            self.get_logger().info('published with offset no. %2i of %2i [%5.2f,%5.2f,%5.2f]'%(i+1,self.nr_of_steps,offset[0],offset[1],offset[2]))


    def publish(self,points,offset):
        """publish"""
        pcd = self.point_cloud(points, self.frame_id_scan)

        version_msg = std_msgs.Int64()
        version_msg.data = self.map_version
        self.version_publisher.publish(version_msg)

        pose_msg = geometry_msgs.PoseStamped()
        pose_msg.header.stamp = pcd.header.stamp
        pose_msg.header.frame_id = self.frame_id_pose
        pose_msg.pose.position.x = -offset[0]
        pose_msg.pose.position.y = -offset[1]
        pose_msg.pose.position.z = -offset[2]
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.pose_publisher.publish(pose_msg)

        self.pcd_publisher.publish(pcd)
        self.ground_scan_publisher.publish(pcd)


    def add_offset(self,points,offset):
        """Add 3D offset to all points"""
        points_new = np.zeros_like(points)
        for i in range(points.shape[0]):
            points_new[i] = points[i] + offset
        return points_new


    def point_cloud(self,points, frame_id):
        """Convert np array of point to sensor_msgs.PointCloud2"""

        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header(frame_id=frame_id)
        header.stamp = Clock().now().to_msg()

        return sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )


def main(args=None):
    """main"""
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
