


# Launch gazebo simulation and control the robot

## Configuring and launching the robot

### To launch the robot

```
ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py
```

----

## RPM Control

### To move the robot through terminal

```
ros2 topic pub --once /rpm_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

### To stop the robot through terminal

```
ros2 topic pub --once /rpm_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

----

## FPM Control

### To move the lift upwards

```
ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1.0"
```

### To bring back the lift to its original position

```
ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0"
```

----

## UR Control

### Send cyclic commands to the UR16 (move all joints of the robot arm)

```
ros2 launch ur_demo ur16_cyclic_joints_traj_cmd.launch.py
```
