# Package fpm_demo

Demonstrate dedicated functions of the Fine Position Module (FPM)

## Documentation

tbd


## HowTo Run


1. **Stand alone lifting unit joint command demo:**

![Visualization](doc/img/Screenshot%20from%202022-08-15%2011-59-10.png)

Load robot description, start Gazebo simulation, spawn lifting unit and load controller.

> ros2 launch fpm_gazebo_simulation fpm_lift_only_gazebo.launch.py

Open another terminal and send controller commands:
    
> ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 2.0"

Or use the demo node to send 4 different positons commands to the forward controller.

> ros2 launch fpm_demo lifting_unit_ctrl.launch.py

2. **Stand alone handling unit joint command demo:**

Load robot description, start Gazebo simulation, spawn handling unit and load controller.

> ros2 launch fpm_gazebo_simulation fpm_sim_control.launch.py

Open another terminal and send trajectory controller commands:

> ros2 launch fpm_demo ur16_cyclic_joints_traj_cmd.launch.py

3. **TCP path planning and execution**

Load Gazebo simulation, moveit, use the RVIZ plugin for motionplanning.
> ros2 launch fpm_moveit fpm_sim_moveit.launch.py

<!--
3. **Cartesian tcp position commands**
Load Gazebo simulation, moveit and fpm action server

> ros2 launch fpm_demo ur16_cyclic_pose_cmd.launch.py
-->