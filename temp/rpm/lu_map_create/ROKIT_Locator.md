# Record ROS1 bag for ROKIT Locator

https://docs.ros.org/en/humble/Tutorials/Demos/Rosbag-with-ROS1-Bridge.html

```bash
sudo apt install ros-galactic-ros1-bridge
```

```bash
# Shell A: ROS1 core
source /opt/ros/noetic/setup.bash
roscore
```

```bash
# Shell B:
source /opt/ros/noetic/setup.bash   # ROS1
source /opt/ros/galactic/setup.bash # ROS2
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

```bash
# Shell C:
source /opt/ros/noetic/setup.bash && source ~/rosws/devel/setup.bash # ROS1
rosbag record /ground_scan /scan /version /pose
```

```bash
# Shell D:
source /opt/ros/galactic/setup.bash && source ~/ros2ws/install/setup.bash # ROS2
ros2 run lu_map_create pcd_publisher_for_rokit_node /home/erz2lr/Downloads/28092022_154530-crop.pts
```

## Data

Install ```ts2-download```

https://cr-slam.de.bosch.com/marv/#/collection/bags
user: slam
pw: crae32
https://cr-slam.de.bosch.com/marv/?filter=ewogInJvYm90IjogewogICJvcCI6ICJzdWJzdHJpbmciLAogICJ2YWwiOiAid2FlZ2VsZSIKIH0KfQ==#/collection/bags
