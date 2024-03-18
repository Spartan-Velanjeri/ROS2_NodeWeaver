# Record ROS1 bag for ROKIT Locator

https://docs.ros.org/en/humble/Tutorials/Demos/Rosbag-with-ROS1-Bridge.html

```bash
sudo apt install ros-noetic-pointcloud-to-laserscan
```

```bash
# Shell A: ROS1 core
source /opt/ros/noetic/setup.bash
roscore
```

```bash
# Shell B:
source /opt/ros/noetic/setup.bash
rosbag play BAGFILE.bag -l /TOPIC:=/cloud_in
```

```bash
# Shell C:
rosbag record /scan /pose
```

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
