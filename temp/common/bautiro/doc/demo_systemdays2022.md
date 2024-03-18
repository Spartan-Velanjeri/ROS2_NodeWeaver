# Demos for system days 2022

please checkout all required repos at branch:
**release/systemdays2022** 
using vcstool with systemdays2022.repos

## Gazebo Simulation ###

### Machine setup ###
- cyclondds setup for machine 1 & 2
- machine 1&2 with ROS2/Gazebo
- machine 3 with HCU and rosbridge



### Launch/Start ###

**Machine 1:** Simulation 
- Start gazebo simulation
    `
ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py `

- Start MoveIT

    `ros2 launch bautiro_bringup bautiro_bringup_sim.launch.py`

- Start fpm_cu

    `ros2 launch fpm_cu  fpm_cu.launch.py`


**Machine 2:** CCU

- Start CCU nodes
`ros2 launch ccu_hcu_abstraction plain.launch.py`

- Start rosbridge for HCU
`ros2 launch src/hcu_ccu_host_prototyp/rosbridge/launchfile.xml`

**Machine 3:** HCU
- Start HCU

----
## Localization and Positioning ##

### Machine setup ###
- Localization-Spectra:
	+ IP: 192.168.88.200
- FPM-Spectra:
    + IP: 192.168.88.220


### Launch/Start ###

**Localization Spectra:** 
 - Start with Terminator:
 	+ Start terminator with demo profil
      terminator -l demo_...
    + Start RVIZ in with demo configuration
      rviz2 ... 
    + Start fein localization sequence
      ros2 run lu_fein_localization fein_localization

**FPM Spectra:** FPM

- Start UR driver
`ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=192.168.2.140 launch_rviz:=false
`

- Start MoveIT and RVIZ 
`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur16e launch_rviz:=true`

- Start fpm_action_server
`ros2 launch fpm_cu  fpm_cu.launch.py`

- Load Collision scene manually



**Demo Sequence:** Demo-Ablauf
- Move "Wägele" manually rough to target position
- Move robot arm to measurement pose (configured position 2)
	Command: ros2 action .... ToDo
- Start fine localization sequence
    Command: ros2 run lu_fein_localization fein_localization
- Target position will be shown on fein localization command windows
- Move robot arm to target pose using move absolute action
    Command: ros2 action send_goal ....ToDo

