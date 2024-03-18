# Mai Demo Launch Instructions
 
## Launch Robot in Lab 131 in Gazebo 
ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py

## Launch all nodes required for simulation and Deployment on Bautiro Hardware 
ros2 launch bautiro_bringup bautiro_bringup.launch.py

 
## To launch system using command line
ros2 action send_goal /ccu_start bautiro_ros_interfaces/action/StartCcuBt start:\ true\

 



Currently not used!

## Launch all nodes required for simulation ONLY
ros2 launch bautiro_bringup bautiro_bringup_sim_nodes.launch.py