
# package: **rpm_demo**

This package is her to test **rpm/bautiro** simulation.
It basically just drops a cmd_vel['Twist] command every *n* seconds.
with the values configured at `config/twist_params.yaml`.

## usage

```bash
# launch
ros2 launch rpm_demo rpm_demo_velocity_cmds.launch.py

# run with custom params
ros2 run rpm_demo --ros-args --params-file <path-to>/params.yaml
```

expected output

```bash
[INFO] [...] [twist_message_publisher]: Publishing 5 twists on topic "/rpm_velocity_controller/cmd_vel_unstamped" every 2 s
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x: 10.0, y: 0.0, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.0
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x:  0.0, y: 0.5, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.5
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x:  6.0, y: 0.0, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.0
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x:  4.0, y: 0.0, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.0
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x:  2.0, y: 0.5, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.5
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x: 10.0, y: 0.0, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.0
[INFO] [...] [twist_message_publisher]: Publishing [Twist] -linear-  x:  0.0, y: 0.5, z: 0.0  -angular- x: 0.0, y: 0.0, z: 0.5
```

cli equivalent to - cylic changing values of -

```bash
ros2 topic pub /rpm_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 15.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Links

- [ros2: Using the ros2 param command-line tool](https://docs.ros.org/en/galactic/How-To-Guides/Using-ros2-param.html)
- The topic name `cmd_vel_unstamped` is hard coded in
  [github.com/ros-controls/ros2_controllers/galactic/diff_drive_controller/src/diff_drive_controller.cpp](https://github.com/ros-controls/ros2_controllers/blob/galactic/diff_drive_controller/src/diff_drive_controller.cpp#L34)
  but can be remapped
