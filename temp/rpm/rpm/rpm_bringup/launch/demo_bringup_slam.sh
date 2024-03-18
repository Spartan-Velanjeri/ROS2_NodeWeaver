#! /bin/bash

source /opt/ros/galactic/setup.bash
source ~/cr_slam_galactic/install/setup.bash

echo "Waiting for LIDAR drivers ..."
sleep 5s

# ros2 launch slam_control waegele_bautiro_vlp16_launch.py slam_mode:=localization map:=/home/bautiro/le139_maps/with_z_estimation_and_ground_plane_ouster_FR/map
# ros2 launch slam_control waegele_bautiro_vlp16_launch.py slam_mode:=localization map:=/home/bautiro/le131_maps/leica_scans/28092022_154530-crop/rosbag2_2022_10_19-08_09_54
# ros2 launch slam_control waegele_bautiro_vlp16_launch.py slam_mode:=localization map:=/home/bautiro/le131_maps/leica_scans/28092022_154530-crop/rosbag2_2022_10_19-08_31_47
ros2 launch slam_control waegele_bautiro_vlp16_launch.py slam_mode:=localization map:=/home/bautiro/le131_maps/leica_scans/28092022_154530-crop/rosbag2_2022_10_26-18_12_54
# ros2 launch slam_control waegele_bautiro_vlp16_launch.py slam_mode:=localization map:=/home/bautiro/le131_maps/leica_scans/28092022_154530_bag_1cm_v2