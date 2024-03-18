## Running the Mobile Base Driver
```
ros2 launch rpm_actuators mobile_base_ros_control.launch.py
```

Start a wheel to  communication:
```
ros2 service call /left_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}



```

Send message to DiffDrive Controller
```
ros2 topic pub /diff_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
linear:
	x: 1.0
	y: 0.0
	z: 0.0
angular:
	x: 0.0
	y: 0.0
	z: 0.0"
```

## Using with fake inverters
```
ros2 launch rpm_actuators mobile_base_with_fake_inverters.launch.py
```
