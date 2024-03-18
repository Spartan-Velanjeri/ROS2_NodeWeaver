# MoveIT2 Configuration

## Comparison of working examples:
[UniversalRobot](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)
[Panda](https://github.com/AndrejOrsula/panda_ign_moveit2)

 File|  Parameter | Universal <br> Robot | Panda | IGN UR
----| ----: | --- | ----| ---- |
 gazebo.urdf| Gazebo plugin <br> filename | libign_ros2_control-system.so | ign_ros2_control-system | 
 gazebo.urdf | Gazebo plugin <br> name | ign_ros2_control::IgnitionROS2ControlPlugin | ign_ros2_control::IgnitionROS2ControlPlugin
ros2_control.urdf | ros2_control/plugin | ign_ros2_control/IgnitionSystem | ign_ros2_control/IgnitionSystem
moveit_controller_manager.yaml | controllers_names | scaled_joint_trajectory_controller <br> **joint_trajectory_controller** | **joint_trajectory_controller** <br> gripper_trajectory_names
moveit_controller_manager.yaml | joint_trajectory_controller | action_ns: follow_joint_trajectory <br> type: FollowJointTrajectory <br> default: true | action_ns: follow_joint_trajectory <br> type: FollowJointTrajectory 
ros2_controller.yaml | controller_manager: | update_rate: Hz <br> joint_state_broadcaster <br> io_and_status_controller <br> joint_trajectory_controller | update_rate: 250Hz <br> joint_state_broadcaster <br> joint_trajectory_controller 
ros2_controller.yaml | joint_trajectory_controller: | command_interfaces: position <br> state_interfaces: position, velocity | command_interfaces: effort <br> state_interfaces: position, velocity