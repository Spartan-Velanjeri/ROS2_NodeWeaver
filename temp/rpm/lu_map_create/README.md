# MAP_CREATE

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/lu_map_create/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/lu_map_create/actions/workflows/build.yml)

Convert a room scan with Leica TS60/MS60 into a map for CR-SLAM

## Install

- Install:
- ```pip3 install open3d```

## Run

1. Export e.g. with Leica Infinity the scan data to PTS pointcloud 
2. Run record
    ```Bash
    ros2 bag record /ground_scan /scan /version /pose
    ```
3. Run the converter from pointcloud to sensor_msgs::msg::PointCloud2 
    ```Bash
    ros2 run lu_map_create pcd_publisher_node /home/bautiro/Downloads/scan.pts /home/bautiro/Downloads/ground_scan.pts
    ```
4. If ```pcd_publisher_node``` finished, stop the recording

## Links
https://artifactory.boschdevcloud.com/ui/native/BautiroArtifacts-local/maps_Le131/

[Bosch DevCloud Bautiro](https://artifactory.boschdevcloud.com/ui/repos/tree/General/BautiroArtifacts-local)

https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
