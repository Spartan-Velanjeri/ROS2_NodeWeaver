#!/usr/bin/env python

#import sys
#sys.path.append('/home/mwo7rng/rosws/src/darknet_ros')
import rospy
import numpy as np
import message_filters
from sensor_msgs import point_cloud2
import sensor_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
import geometry_msgs.msg
# from sensor_msgs.msg import Image, CameraInfo
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
import os
import time
# import shapely
# from shapely.geometry import Polygon
# from shapely.ops import cascaded_union
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import os.path as osp
# import Image
import struct

class MapCreate:

    node_name = "map_create"

    ## PARAMETERS #########################################

    tpc_sub_pcl = '/velodyne/velodyne_points'
    # tpc_pub_map = '/' + node_name + '/map' # result 
    tpc_pub_map = '/scan'        # map 
    tpc_pub_version = '/version' # verion of the map 
    tpc_pub_pose = '/pose' # initial pose
    map_version = 3
    buffer_inpt = 30
    buffer_else = 30
    loop_run = True
    USE_APPROXIMATETIMESYNCHRONIZER = True


    #######################################################

    pub_map = rospy.Publisher(tpc_pub_map,sensor_msgs.msg.PointCloud2,queue_size=buffer_else)
    pub_version = rospy.Publisher(tpc_pub_version,Int64,queue_size=buffer_else)
    pub_pose = rospy.Publisher(tpc_pub_pose,geometry_msgs.msg.PoseStamped,queue_size=buffer_else)


    def callback_pcl(self, msg):
        map_msg = self.create_map()
        if self.loop_run:
            map_msg.header.stamp = rospy.Time.now()
        else:
            map_msg.header = msg.header

        version_msg = Int64()
        version_msg.data = self.map_version
        # ROS_ERROR("Maps version 1 and 2 are going to be deprecated soon since they support 2D only. Consider updating them to version 3");

        pose_msg = geometry_msgs.msg.PoseStamped()
        pose_msg.header = map_msg.header
        pose_msg.pose.position.x = 0
        pose_msg.pose.position.y = 0
        pose_msg.pose.position.z = 0
        pose_msg.pose.orientation.w = 1
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0

        self.pub_map.publish(map_msg)
        self.pub_version.publish(version_msg)
        self.pub_pose.publish(pose_msg)
        rospy.loginfo(rospy.get_caller_id() + ' ' + str(map_msg.header.stamp.to_sec()) + ' PointCloud, version and pose published!')


    def __init__(self):
        rospy.init_node(self.node_name, anonymous=False)

        if not self.loop_run:
            rospy.Subscriber(self.tpc_sub_pcl, sensor_msgs.msg.PointCloud2, self.callback_pcl, queue_size=self.buffer_inpt)

        # if self.USE_APPROXIMATETIMESYNCHRONIZER:
        #     max_delay_seconds = 0.1
        #     ats = message_filters.ApproximateTimeSynchronizer([imageL_sub,imageR_sub], self.buffer_sync, max_delay_seconds)
        #     ats.registerCallback(self.callback_datain)
        #     rospy.loginfo(rospy.get_caller_id() + " Using ApproximateTimeSynchronizer.")
        # else:
        #     ts = message_filters.TimeSynchronizer([imageL_sub,imageR_sub], self.buffer_sync)
        #     ts.registerCallback(self.callback_datain)
        #     rospy.loginfo(rospy.get_caller_id() + " Using TimeSynchronizer.")

        if self.loop_run:
            rospy.loginfo(rospy.get_caller_id() + ' Node running ...')
            while not rospy.is_shutdown():
                self.callback_pcl(sensor_msgs.msg.PointCloud2())
                rospy.sleep(1.0)
        else:
            rospy.loginfo(rospy.get_caller_id() + ' Node running, waiting for topics ...')
            rospy.spin()


    def create_map(self):
        resolv = 30
        noise = 3

        #=====#
        x_ = np.arange(0,73510,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data = np.concatenate((-1*x+x_noise,y+y_noise), axis=1)

        #=====#
        y_ = np.arange(resolv,2280,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],-1*y+y_noise+data[-1,1]), axis=1) #[x+x_noise+data[-1,0],-1*y+y_noise+data(2,end)]
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_ = np.arange(resolv,40000,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        y_ = np.arange(resolv,5370,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],-1*y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_ = np.arange(resolv,3500,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        y_ = np.arange(resolv,5370,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_ = np.arange(resolv,19900,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        y_ = np.arange(resolv,49130,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],-1*y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_ = np.arange(resolv,2300,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        y_ = np.arange(resolv,49130,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_ = np.arange(resolv,7690,resolv)
        x = np.reshape(x_,(x_.shape[0],1))
        y = np.zeros(x.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        y_ = np.arange(resolv,2280,resolv)
        y = np.reshape(y_,(y_.shape[0],1))
        x = np.zeros(y.shape)
        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((x+x_noise+data[-1,0],y+y_noise+data[-1,1]), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_c = 8050
        y_c = 250
        r = 350/2
        phi_ = np.arange(1,2*np.pi,2*np.pi/36)
        phi = np.reshape(phi_,(phi_.shape[0],1))
        x = r*np.cos(phi) + x_c
        y = r*np.sin(phi) + y_c

        x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
        y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
        data_section = np.concatenate((-1*x+x_noise,-1*y+y_noise), axis=1)
        data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_c = 8050
        y_c = 250
        r = 350/2

        for i in range(1,9):
            x_c = x_c + 7200
            x = r*np.cos(phi) + x_c
            y = r*np.sin(phi) + y_c

            x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
            y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))
            data_section = np.concatenate((-1*x+x_noise,-1*y+y_noise), axis=1)
            data = np.concatenate((data,data_section), axis=0)

        #=====#
        x_c = 8050
        y_c = 250
        r = 350/2

        for i in range(1,7):
            y_c = y_c + 7200
            x = r*np.cos(phi) + x_c
            y = r*np.sin(phi) + y_c

            x_noise = noise*(2*(np.random.rand(x.shape[0],1)-0.5))
            y_noise = noise*(2*(np.random.rand(y.shape[0],1)-0.5))    
            data_section = np.concatenate((-1*x+x_noise,-1*y+y_noise), axis=1)
            data = np.concatenate((data,data_section), axis=0)

        # #=====# Plot

        # subsample = 1
        # plot(data(1,1:subsample:end)/1e3,data(2,1:subsample:end)/1e3,'.k')
        # axis equal
        # xlabel('x [m]')
        # ylabel('y [m]')
        # grid on

        #=====# Copy to MSG
        header = Header()
        header.frame_id = "velodyne"

        fields = [sensor_msgs.msg.PointField('x', 0, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('y', 4, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('z', 8, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('rgba', 12, sensor_msgs.msg.PointField.UINT32, 1),
                 ]

        points = []
        for i in range(data.shape[0]):
            rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
            pt = [data[i,0]/1e3, data[i,1]/1e3, 0.0, rgb]
            points.append(pt)

        msg_pc2 = point_cloud2.create_cloud(header,fields, points)

        return msg_pc2

    ####################################################################################################################

if __name__ == '__main__':
    cl = MapCreate()

