#! /usr/bash

# rosbag play -l /mnt/c/daten/messungen/waegele/2021-05-06-16-51-46.bag --clock
# rviz -d $(rospack find map_create)/cfg/config.rviz

# Record MAP BAG file ##########################################################
cd ~/tmp/bag_slam_gt
ros2 bag record /scan /version /pose
ros2 launch pcd_demo pcd_publisher.launch.py

# Run CR-SLAM #################################################################
roslaunch slam_control waegele_vlp16.launch bag:=/mnt/c/daten/messungen/waegele/2021-05-06-16-51-46.bag

# Run CR-SLAM in localisation mode #############################################
rosrun rviz rviz -d $(rospack find slam_control)/rviz/slam.rviz
FILE_SCANS=/mnt/c/daten/messungen/waegele/2021-05-06-16-51-46.bag
FILE_MAP= /home/erz2lr/tmp/bag_slam_gt/map_2021-08-18-11-44-13_from_2021-05-06-16-51-46_version2.bag
roslaunch slam_control waegele_vlp16.launch bag:=${FILE_SCANS} map:=${FILE_MAP} slam_mode:=localization

# ERRORs #######################################################################
[FATAL] [1629278959.291636200]: [LocalizationGroundTruthHandling] Map does not contain any ground truth vertices.

# INFOs
[ INFO] [1629278959.231676000]: [config] Using modified parameter for key 'localization/ground_truth_handling/enable': '1'
[ INFO] [1629278959.231945800]: [config] Using default parameter for key 'localization/ground_truth_handling/use_adma': '0'
[ INFO] [1629278959.232410700]: [config] Using modified parameter for key 'localization/ground_truth_handling/shall_estimate_start_pose_from_bag': '0'
[ INFO] [1629278959.232677100]: [config] Using default parameter for key 'localization/ground_truth_handling/use_ground_truth_as_start_pose': '0'
