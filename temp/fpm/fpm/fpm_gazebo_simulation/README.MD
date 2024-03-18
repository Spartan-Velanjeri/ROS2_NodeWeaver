# Package fpm_gazebo_simulation

Simulate the FPM using Gazebo Fortress.

### Ignition Gazebo, ROS2 Control and MoveIT Architecture
![Architecture](../doc/img/ros_control_gazebo.drawio.svg)



### Instructions 

Prepare Xacro (URDF) for Ignition:
https://github.com/ros-controls/gz_ros2_control/tree/galactic
- Add gazebo tag to a URDF
   

ROS2 control:
- controller config *.yaml
- Add ros2_control tag/ load libs to interface Ignition using ROS2 Control to the URDF
    - GZ_SIM https://github.com/gazebosim/gz-sim
    - ign_ros2_control 


### Run Simulations

1. Stand alone lifting unit demo:

    Load robot description, start Gazebo simulation, spawn lifting unit and load controller.
    >ros2 launch fpm_gazebo_simulation fpm_lift_only_gazebo.launch.py

    Open another terminal and send controller commands:
    ```
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 2.0"

2. Stand alone handling unit demo:

    ```
    ros2 launch fpm_gazebo_simulation fpm_ur16_only_gazebo.launch.py