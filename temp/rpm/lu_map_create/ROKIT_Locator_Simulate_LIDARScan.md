# Simulierte Punktewolke als Lidar Messung

1. ROS2 bag mit PCD aufnehmen

    ros2 bag record /scan /pose --max-cache-size 4000000

    ```bash
    # Shell D:
    source /opt/ros/galactic/setup.bash && source ~/ros2ws/install/setup.bash # ROS2
    ros2 run lu_map_create pcd_publisher_for_rokit_node /home/erz2lr/Downloads/28092022_154530-crop.pts
    ```

2. ros2 nach ros2 bag convertieren mit: ```rosbags-convert```

3. Aus PCD eine LaserScan erzeugen
    ```bash
    # Shell D:
    source /opt/ros/noetic/setup.bash
    rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node \
        _min_height:=1.0 \
        _max_height:=1.2 \
        _angle_increment:=0.000872639 \
        _range_min:=0.01 \
        _range_max:=40.0
    ```
