#!/usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
import yaml
import io
import os
import time

import rclpy
from rclpy.node import Node
from bautiro_ros_interfaces.srv import GetTrafo

from ament_index_python.packages import get_package_share_directory

from lu_rough_localization.process_ros_parameters import process_ros_parameters


class ManipulateYaml(Node):

    # Launch arguments
    launch_args = {
        "config_template_general": {
            "description": "tbd",
            "default": "fus1_slam_general_sim.yaml",
            "value": ''
        },
        "config_template_lidar": {
            "description": "tbd",
            "default": "fus1_slam_lidar_sim.yaml",
            "value": ''
        },
    }

    # Frame name for the base link
    from_frame = 'base_link'

    # Frame names of the sensors in TF tree
    to_frame_lidar_front = 'rpm_lidar_front_lidar'  # on Waegele:'ouster_front_left'
    to_frame_lidar_rear = 'rpm_lidar_rear_lidar'  # on Waegele: 'ouster_rear_right'
    to_frame_imu = 'rpm_imu'  # on Waegele: 'xsens_imu'

    # Entry in the YAML config file
    yaml_tmp_path = '/tmp/'
    out_yaml_file_general = 'slam_general.yaml'
    out_yaml_file_lidar = 'slam_lidar.yaml'
    entry_str_imu = "tf/base_link_to_imu"
    entry_str_lidar_front = 'ouster_F/base_link_to_sensor'
    entry_str_lidar_rear = 'ouster_R/base_link_to_sensor'

    def __init__(self):
        super().__init__('manipulate_yaml')

        self.launch_args = process_ros_parameters(self, self.launch_args)
        yaml_file_general = self.launch_args["config_template_general"]["value"]
        yaml_file_lidar = self.launch_args["config_template_lidar"]["value"]

        yaml_path = os.path.join(
            get_package_share_directory('lu_rough_localization'),
            'config')
        self.get_logger().info("Using %s ..." % (yaml_file_general))
        self.get_logger().info("Using %s ..." % (yaml_file_lidar))

        if os.path.isfile(self.yaml_tmp_path + yaml_file_general):
            os.system('rm ' + self.yaml_tmp_path + yaml_file_general)

        if os.path.isfile(self.yaml_tmp_path + yaml_file_lidar):
            os.system('rm ' + self.yaml_tmp_path + yaml_file_lidar)

        # create and initialize client for transformation service /lu_get_trafo
        self.trafo_client = self.create_client(GetTrafo, '/lu_get_trafo')
        while not self.trafo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Service LU_GET_TRAFO_GENERAL not available, waiting ...')
        self.trafo_client_req = GetTrafo.Request()

        # Check for robot description
        is_repeat = True
        while is_repeat:
            print('Checking for robot description '
                  '(base_link --> rpm_lidar_front_lidar) ...')
            trans = self.call_trafo('base_link', 'rpm_lidar_front_lidar')
            if trans.pose.position.x != 0 or \
                    trans.pose.position.y != 0 or \
                    trans.pose.position.z != 0:
                is_repeat = False
                self.get_logger().info('Robot description available.')
            else:
                self.get_logger().info('Waiting for robot description '
                                       '(base_link --> rpm_lidar_front_lidar) ...')
                time.sleep(1.0)

        # Call TF tree
        self.trafo_to_imu = self.call_trafo(
            self.from_frame, self.to_frame_imu)
        self.trafo_to_lidar_front = self.call_trafo(
            self.from_frame, self.to_frame_lidar_front)
        self.trafo_to_lidar_rear = self.call_trafo(
            self.from_frame, self.to_frame_lidar_rear)

        self.change_imu_calibration(
            os.path.join(yaml_path, yaml_file_general),
            self.yaml_tmp_path + self.out_yaml_file_general)
        self.change_lidar_calibration(
            os.path.join(yaml_path, yaml_file_lidar),
            self.yaml_tmp_path + self.out_yaml_file_lidar)
        self.get_logger().info("Created temporary config files for SLAM.")

    def call_trafo(self, from_str, to_str):
        self.trafo_client_req.from_frame.data = from_str
        self.trafo_client_req.to_frame.data = to_str
        self.resp = self.trafo_client.call_async(self.trafo_client_req)
        rclpy.spin_until_future_complete(self, self.resp)
        return self.resp.result().out_pose

    def change_imu_calibration(self, yaml_file_in, yaml_file_out):
        # Change the following information in the file
        # tf/base_link_to_imu: [0.724, -0.353, 0.653, 0.000, 0.000, 0.000, 1.000]

        # Read YAML file
        with open(yaml_file_in, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        # Change the value
        data_loaded[self.entry_str_imu][0] = self.trafo_to_imu.pose.position.x
        data_loaded[self.entry_str_imu][1] = self.trafo_to_imu.pose.position.y
        data_loaded[self.entry_str_imu][2] = self.trafo_to_imu.pose.position.z
        data_loaded[self.entry_str_imu][3] = self.trafo_to_imu.pose.orientation.x
        data_loaded[self.entry_str_imu][4] = self.trafo_to_imu.pose.orientation.y
        data_loaded[self.entry_str_imu][5] = self.trafo_to_imu.pose.orientation.z
        data_loaded[self.entry_str_imu][6] = self.trafo_to_imu.pose.orientation.w

        # Write YAML file
        with io.open(yaml_file_out, 'w', encoding='utf8') as outfile:
            yaml.dump(data_loaded, outfile, default_flow_style=False, allow_unicode=True)

    def change_lidar_calibration(self, yaml_file_in, yaml_file_out):
        # Change the following information in the file
        # tf/base_link_to_imu: [0.724, -0.353, 0.653, 0.000, 0.000, 0.000, 1.000]

        # Read YAML file
        with open(yaml_file_in, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        # Change the value
        data_loaded[self.entry_str_lidar_front][0] = self.trafo_to_lidar_front.pose.position.x
        data_loaded[self.entry_str_lidar_front][1] = self.trafo_to_lidar_front.pose.position.y
        data_loaded[self.entry_str_lidar_front][2] = self.trafo_to_lidar_front.pose.position.z
        data_loaded[self.entry_str_lidar_front][3] = self.trafo_to_lidar_front.pose.orientation.x
        data_loaded[self.entry_str_lidar_front][4] = self.trafo_to_lidar_front.pose.orientation.y
        data_loaded[self.entry_str_lidar_front][5] = self.trafo_to_lidar_front.pose.orientation.z
        data_loaded[self.entry_str_lidar_front][6] = self.trafo_to_lidar_front.pose.orientation.w

        data_loaded[self.entry_str_lidar_rear][0] = self.trafo_to_lidar_rear.pose.position.x
        data_loaded[self.entry_str_lidar_rear][1] = self.trafo_to_lidar_rear.pose.position.y
        data_loaded[self.entry_str_lidar_rear][2] = self.trafo_to_lidar_rear.pose.position.z
        data_loaded[self.entry_str_lidar_rear][3] = self.trafo_to_lidar_rear.pose.orientation.x
        data_loaded[self.entry_str_lidar_rear][4] = self.trafo_to_lidar_rear.pose.orientation.y
        data_loaded[self.entry_str_lidar_rear][5] = self.trafo_to_lidar_rear.pose.orientation.z
        data_loaded[self.entry_str_lidar_rear][6] = self.trafo_to_lidar_rear.pose.orientation.w

        # Write YAML file
        with io.open(yaml_file_out, 'w', encoding='utf8') as outfile:
            yaml.dump(data_loaded, outfile, default_flow_style=False, allow_unicode=True)


def main(args=None):
    rclpy.init(args=args)

    my = ManipulateYaml()
    rclpy.spin_once(my)


if __name__ == '__main__':
    main()
