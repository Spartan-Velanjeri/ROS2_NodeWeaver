
"""
https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md
"""

import sys
import os
from time import sleep

import rclpy 
from rclpy.node import Node
from rclpy.clock import Clock
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
# from std_msgs.msg import Int64v
import nav_msgs.msg as nav_msgs

import numpy as np
import open3d as o3d

class trafo():

    def __init__(self,pcd,resolution):
        self.resolution = resolution

        self.range_precise = np.zeros(shape=(2,2))
        self.range = np.zeros(shape=(2,2))
        self.m = np.zeros(shape=(2,1))
        self.n = np.zeros(shape=(2,1),dtype=int)
        self.off = np.zeros(shape=(2,1))
        
        self.get_trafo_params(pcd)


    def get_trafo_params_for_one_dim(self,points,idx):
        self.range_precise[idx] = np.array((points[:,idx].min(),points[:,idx].max()))

        if self.resolution < 1.0:
            self.range[idx] = self.range_precise[idx].round(int(abs(np.floor(np.log10(self.resolution)))))
        else:
            self.range[idx] = self.range_precise[idx].round(int(abs(np.ceil(np.log10(self.resolution)))))

        self.m[idx] = 1.0/self.resolution
        self.n[idx] = int(self.m[idx]*np.diff(self.range[idx]))
        self.off[idx] = self.n[idx] - self.m[idx]*self.range[idx][1]        
        return


    def get_trafo_params(self,points):
        for i in range(2):
            self.get_trafo_params_for_one_dim(points,i)
        
        return


    def get_coord(self,point):
        coord = np.zeros(2)
        for idx in range(2):
            coord[idx] = self.m[idx] * point[idx] + self.off[idx]
        coord = coord.astype(int)
        return coord



class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_for_rokit_node')

        assert len(sys.argv) > 1, "No pts file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        pts_path = sys.argv[1]

        # Convert with Open3D in numpy array
        pcd = o3d.io.read_point_cloud(pts_path)
        self.points = np.asarray(pcd.points)
        self.get_logger().info('Read PTS file with %i %iD-points.'%(self.points.shape[0],self.points.shape[1]))
        # print(self.points.shape)

        # Publisher        
        # self.version_publisher = self.create_publisher(std_msgs.Int64, 'version', 10)
        self.pcd_scan_publisher     = self.create_publisher(sensor_msgs.PointCloud2, 'scan', 10)
        self.pcd_publisher     = self.create_publisher(nav_msgs.OccupancyGrid, 'occupancygrid', 10)
        # self.ground_scan_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'ground_scan', 10)

        self.frame_id_scan = 'map'              # frame ID
        self.resolution = 0.10                  # resolution of the occupancy grid in meters
        self.height_range = np.array((0.5,3.0)) # height range to be considered in meters


        # Publish twice
        sleep(3.0)
        self.run()
        sleep(1.0)


    def run(self):
        """publish"""
        self.get_logger().info('Converting pointcloud to occupancy grid ...')
        self.get_logger().info('> frame_id_scan = %s'%self.frame_id_scan)
        self.get_logger().info('> resolution    = %.2fm'%self.resolution)
        self.get_logger().info('> height_range  = %.2fm - %.2fm'%(self.height_range[0],self.height_range[1]))

        pcd = self.occupancygrid(self.points, self.frame_id_scan)
        self.get_logger().info('Publishing occupancy grid ...')
        self.pcd_publisher.publish(pcd)
        self.get_logger().info('Publishing pointcloud...')
        self.scan(self.points, self.frame_id_scan)


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


    def scan(self,points,frame_id):
        """publish"""
        pcd = self.point_cloud(points, frame_id)
        self.pcd_scan_publisher.publish(pcd)


    def occupancygrid(self,points, frame_id):
        """Convert np array of point to nav_msgs.OccupancyGrid"""

        tf = trafo(points,self.resolution)

        origin = geometry_msgs.Pose()
        origin.position.x = tf.range_precise[0,0]
        origin.position.y = tf.range_precise[1,0]
        origin.position.z = 0.0
        origin.orientation.x = 0.0
        origin.orientation.y = 0.0
        origin.orientation.z = 0.0
        origin.orientation.w = 1.0
        
        img = np.zeros(shape=(tf.n + 1).transpose().tolist()[0],dtype=np.int8)
        for point in points:
            if point[2] < self.height_range[0] or point[2] > self.height_range[1]:
                continue
            c = tf.get_coord(point)
            if img[c[0],c[1]] < 100:
                img[c[0],c[1]] = img[c[0],c[1]] + 1

        map_msg = nav_msgs.OccupancyGrid()
        map_msg.header.frame_id = frame_id
        map_msg.header.stamp    = Clock().now().to_msg()
        map_msg.info.resolution = self.resolution         # float32
        map_msg.info.width      = img.shape[0]           # uint32
        map_msg.info.height     = img.shape[1]           # uint32
        map_msg.info.origin     = origin
        map_msg.data            = img.ravel(order='F').tolist()

        return map_msg


def main(args=None):
    """main"""
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
