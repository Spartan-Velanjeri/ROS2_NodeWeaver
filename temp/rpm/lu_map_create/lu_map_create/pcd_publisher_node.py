
import sys
import os
from time import sleep

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
# from std_msgs.msg import Int64

import numpy as np
import open3d as o3d

class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_node')

        assert len(sys.argv) > 2, "Two PTS file are required as input parameters."
        assert os.path.exists(sys.argv[1]), "The 1st PTS file doesn't exist."
        assert os.path.exists(sys.argv[2]), "The 2nd PTS file doesn't exist."
        pts_path = sys.argv[1]
        pts_ground_path = sys.argv[2]

        # Convert with Open3D in numpy array
        pcd = o3d.io.read_point_cloud(pts_path)
        pcd_ground = o3d.io.read_point_cloud(pts_ground_path)

        # Move the PCD origin to zero height
        bound_min_ground = pcd_ground.get_min_bound()
        bound_min_walls  = pcd.get_min_bound()
        if bound_min_walls[2] >=  bound_min_walls[2]:
            bound_min = bound_min_ground[2]
            pcd_ground = pcd_ground.translate((0.0,0.0,-1*bound_min))
            pcd = pcd.translate((0.0,0.0,-1*bound_min))
            print("Moving the point cloud to %f m."%(-1*bound_min))
        else:
            print("ERROR: minimum value of the walls is below the ground lowest point!!!")
            return 1

        # Convert to NumPy array
        self.points = np.asarray(pcd.points)
        self.points_ground = np.asarray(pcd_ground.points)
        print("%i points for the walls and %i for the ground."%(self.points.shape[0],self.points_ground.shape[0]))
        

        # Publisher        
        self.version_publisher = self.create_publisher(std_msgs.Int64, 'version', 10)
        self.pose_publisher    = self.create_publisher(geometry_msgs.PoseStamped, 'pose', 10)
        self.pcd_publisher     = self.create_publisher(sensor_msgs.PointCloud2, 'scan', 10)
        self.ground_scan_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'ground_scan', 10)

        self.map_version = 3
        self.frame_id = 'map'

        # Publish twice
        self.run()
        sleep(2.0)
        self.run()

              
                
    def run(self):
        self.pcd = point_cloud(self.points, self.frame_id)
        self.pcd_ground = point_cloud(self.points_ground, self.frame_id)
        self.pcd_publisher.publish(self.pcd)
        self.ground_scan_publisher.publish(self.pcd_ground)

        pose_msg = geometry_msgs.PoseStamped()
        pose_msg.header = self.pcd.header
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.pose_publisher.publish(pose_msg)

        version_msg = std_msgs.Int64()
        version_msg.data = self.map_version
        self.version_publisher.publish(version_msg)


def point_cloud(points, frame_id):

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
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
#    rclpy.spin(pcd_publisher)
    
    sleep(1.0)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
