#!/usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

"""Filter for LIDAR point cloud."""
import numpy as np
import time
# import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

# from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, PointField

from lu_rough_localization import point_cloud2 as pcd2

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

is_visualize = False
is_printout = False

if is_visualize:
    import open3d as o3d


class PcdFilter(Node):
    """Point cloud filter."""

    def __init__(self):
        node_name = 'filter_outliers_%i' % (int(np.random.random() * 1e4))
        super().__init__(node_name)

        self.declare_parameter('tpc_in', '/rpm/sensors/front/lidar3d/points')
        # self.declare_parameter('tpc_out', '/rpm/sensors/front/lidar3d/points_filtered')
        self.declare_parameter('buffer', 2)
        self.declare_parameter('max_distance', 14.0)
        self.declare_parameter('closest_neighbor_distance', 0.05)

        self.tpc_in = self.get_parameter('tpc_in').value
        # self.tpc_out = self.get_parameter('tpc_out').value
        self.buffer = self.get_parameter('buffer').value
        self.max_distance = self.get_parameter('max_distance').value
        self.closest_neighbor_distance = self.get_parameter(
            'closest_neighbor_distance').value

        # Subscriber
        self.get_logger().info('Subscribing to %s ...' % (self.tpc_in))
        self.sub_pcd = self.create_subscription(
            PointCloud2, self.tpc_in, self.callback_filter_pcd, self.buffer)
        # self.sub_scan = self.create_subscription(
        #     LaserScan, self.tpc_in.replace('points','scan'),
        #     self.filter_pcd_2, self.buffer)
        # self.sub_rangeimage = self.create_subscription(
        #     Image, self.tpc_in.replace('points','intensity_image'),
        #     self.filter_range_image, self.buffer)

        # Publisher
        self.tpc_out = self.tpc_in + '_filtered'
        self.pub_pcd = self.create_publisher(PointCloud2, self.tpc_out, self.buffer)

        # if is_visualize:
        #     self.vis = o3d.visualization.Visualizer()
        #     self.vis.create_window(width=1400, height=700)
        #     self.cam_parameters = o3d.io.read_pinhole_camera_parameters("cfg/ScreenCamera.json")

        self.layer = 0
        self.neighbor = np.zeros((3, 3))  # [(0,0,0),(0,0,0),(0,0,0)]
        self.computation_time = 0.0
        self.computation_cnt = 0

        self.get_logger().info('filter_outliers ready and waiting for commands ...')

    def diff_row(self):
        return np.max(np.abs(np.diff(self.neighbor, axis=0)))

    def callback_filter_pcd(self, msg):
        '''Point cloud outlier removal by simple neighbor comparison.'''

        t0_all = time.time()

        assert len(msg.fields) >= 4, "The pointcloud should contain at least x,y,z,intensity"

        # n = int(msg.height*msg.width)
        # n_fields = 4
        t0 = time.time()
        pcd_in__np = pcd2.pointcloud2_to_array(msg, leave_one_dimensional=True)
        pcd_in = pcd2.get_xyzi_points(pcd_in__np)
        t1 = time.time()
        if is_printout:
            print("========================")
            print("NEW msg --> numpy   %f" % (t1 - t0))

        # sparial filter
        t0 = time.time()

        max_dist = np.max(pcd_in[:, 0:3].__abs__(), axis=1)

        sel_pre = pcd_in > 0
        sel = np.logical_or(np.logical_or(
            sel_pre[:, 0], sel_pre[:, 1]), sel_pre[:, 2])
        sel = np.logical_and(sel, max_dist < self.max_distance)

        pcd_out__np_ = pcd_in[sel]

        dist = np.max(np.abs(np.diff(pcd_out__np_[:, 0:3], axis=0)), axis=1) / (max_dist[sel])[1:]
        sel_diff = np.logical_and(dist[0:-1] < self.closest_neighbor_distance,
                                  dist[1:] < self.closest_neighbor_distance)
        pcd_out__np__ = (pcd_out__np_[2:, :])[sel_diff]

        t1 = time.time()
        if is_printout:
            print("Spatial filter      %f" % (t1 - t0))

        # Convert to msg and publish
        t0 = time.time()
        pcd_msg = self.point_cloud_xyzi(
            msg.header, msg.fields[0:4],
            pcd_out__np__, msg.is_bigendian, msg.is_dense)
        t1 = time.time()
        if is_printout:
            print("NumPy->Msg          %f" % (t1 - t0))

        if is_visualize:
            print("Showing input (red) and output (gray): ")

            pcd_in__np = np.array(list(self.read_points(msg)))
            pcd_in__o3d = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pcd_in__np[:, 0:3]))

            pcd_out__np = np.array(list(self.read_points(pcd_msg)))
            pcd_out__o3d = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pcd_out__np[:, 0:3]))

            pcd_in__o3d.paint_uniform_color([1, 0, 0])
            pcd_out__o3d.paint_uniform_color([0.8, 0.8, 0.8])
            self.vis.add_geometry(pcd_in__o3d)
            self.vis.add_geometry(pcd_out__o3d)

            ctr = self.vis.get_view_control()
            ctr.convert_from_pinhole_camera_parameters(self.cam_parameters)

            self.vis.poll_events()
            self.vis.update_renderer()

            # clear visualization
            self.vis.remove_geometry(pcd_in__o3d)
            self.vis.remove_geometry(pcd_out__o3d)

        self.pub_pcd.publish(pcd_msg)

        # Printout computation time
        t1_all = time.time()
        t_diff_all = t1_all - t0_all
        self.computation_cnt += 1
        self.computation_time = (self.computation_time * (self.computation_cnt - 1) + t_diff_all) \
            / self.computation_cnt
        if self.computation_cnt % 40 == 0:
            self.get_logger().info("Running with %.1fms ..." % (self.computation_time * 1e3))
            self.computation_cnt = 0

    def point_cloud_xyzi(self, header, fields, points, is_bigendian, is_dense):
        """
        Convert NumPy array to PointCloud2.

        header       - \\
        fields       - \\
        points       - NumPy array with data \\
        is_bigendian - \\
        is_dense     - if is set to True, assumes that all points are valid.
        """

        N = len(points)
        # ros_dtype = PointField.FLOAT32
        dtype = np.float32
        # itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        itemsize = []  # Every point consists of x float32s.
        fields_new = []
        offset = 0
        for field in fields:
            t = _DATATYPES[field.datatype][1]
            itemsize.append(t)
            fields_new.append(
                PointField(name=field.name, offset=offset,
                           datatype=field.datatype, count=1))
            offset = offset + t

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represents. The first 4 bytes
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        # fields_new = [
        #     PointField(name=fields.name, offset=i*itemsize,
        #                datatype=ros_dtype, count=1) for i, n in fields]

        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        # header = Header(frame_id=frame_id)

        return PointCloud2(
            header=header,
            height=1, width=N,  # put all points in one row
            is_dense=is_dense,
            is_bigendian=is_bigendian,
            fields=fields_new,
            point_step=sum(itemsize),
            row_step=(sum(itemsize) * N),
            data=data
        )


def main(args=None):
    rclpy.init(args=args)

    node = PcdFilter()
    # node.th.join()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
