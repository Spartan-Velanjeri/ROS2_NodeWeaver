# FPM handling unit

The handling unit for the PoC is a Universal-Robot UR16e arm.
Motion control is done by the arm controller. Movement is separated in two steps:

- **Positioning**: Moving the arm for positioning purpose only. Planing and execution is done using ROS2 MoveIT
- **Drilling**: Movement for drilling holes: Low level control with force mode using the arm controller, high level monitoring and commands by the fpm coordinator.

## ROS2 structure of nodes and configurations related to the handling unit

![Overview of Control architecture using ROS2](img/sw_architecture_ros2_moveit.drawio.svg)
