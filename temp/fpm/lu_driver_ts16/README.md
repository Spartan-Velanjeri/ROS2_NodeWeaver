# ROS2 Driver Leica TS16

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/lu_driver_ts16/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/lu_driver_ts16/actions/workflows/build.yml)

## Installation

```bash
sudo apt install python3-serial
sudo usermod -aG tty ${USER}
sudo usermod -aG dialout ${USER}
```


## Run

Verify interface settings:

* in Settings 
  * Connections
    * All other connections
      * GeoCOM
        * Set "Connect using" select Cable
        * Verify the IP address and port
* Meas&Stream
  * Fn
    * Settings
      * Interface
        * Connect using = "Configured interface"
        * Interface = "GeoCOM"
        * Format = "Pt, E, N, Ht, date"
* your computer needs to have a 192.168.254.X IP address on the USB ethernet adapter

Verify tracking settings:

* Meas & Stream app is started
* Correct prism type is set (usually Leica Miniprism)
* Measurement mode is set to continuous

Notes on the output:
Since the data from the MS16 is subject to heavy jitter three topics are published

* position: using the measurement timestamps in the message header and drift compensation (default)
* pub_position_no_drift_compensation: using the measurement timestamps in the message header (default)
* position_raw_timing: using the ROS timestamps on data reception in the message header
* timeref: translates between both above timestamps

Example for a target position as coordinates:

```bash
ros2 topic pub --once /target_position_coord geometry_msgs/msg/PointStamped "{point: {x: 1.1,y: 1.2, z: 1.3}}"
ros2 topic pub --once /target_position_coord geometry_msgs/msg/PointStamped "{point: {x: 4.4811, y: -1.4931, z: 1.1987}}"
```

or

```bash
ros2 service call /fpm/sensors/ts16/lu_control_ts16 bautiro_ros_interfaces/srv/LuTsMeasurePoint "in_point: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, point: {x: -4.0, y: 1.96, z: -1.0}}"
```

or

```bash
ros2 service call /lu_control_ts16_multi bautiro_ros_interfaces/srv/LuControlTs16Multi "in_point: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, point: {x: -4.67, y: 1.96, z: 1.19}},{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, point: {x: 1.0, y: 0.0, z: 0.0}},{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, point: {x: -4.09, y: 2.51, z: 1.19}}]"
```

## Usage of the service

```py
from bautiro_ros_interfaces.srv import LuControlTs16

# create and initialize client for ts16 control service
self.control_ts16_client = self.create_client(LuControlTs16, '/fine/ts16/lu_control_ts16')
while not self.control_ts16_client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service to control ts16 not available, waiting again...')
self.control_ts16_req = LuControlTs16.Request()

self.control_ts16_req.in_point = marker_rough_position
self.future = self.control_ts16_client.call_async(self.control_ts16_req)
rclpy.spin_until_future_complete(self, self.future)

print(self.future.result().out_point)
```
