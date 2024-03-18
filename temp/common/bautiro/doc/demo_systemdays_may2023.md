# Demo 05/2023

## Prepare your workspace with these commands

git clone -b maydemo2023 ssh://git@sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro.git  


cd $DEVDIR  
vcs import src < src/bautiro/dependencies.repos  
vcs import src < src/fpm/dependencies.repos  
vcs import src < src/universal_robots_ros2_driver/Universal_Robots_ROS2_Driver.galactic.repos  
vcs import src < src/moveit2/moveit2.repos  
vcs import src < src/rpm/dependencies.repos  

d
## Mission execution simulation

### Launch Robot in Lab 131 in Gazebo 
ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py

### Launch all nodes required for simulation and Deployment on Bautiro Hardware 
ros2 launch bautiro_bringup bautiro_bringup.launch.py

 
### To launch system using command line
ros2 action send_goal /ccu_start bautiro_ros_interfaces/action/StartCcuBt start:\ true\

 



Currently not used!

### Launch all nodes required for simulation ONLY
ros2 launch bautiro_bringup bautiro_bringup_sim_nodes.launch.py


## Positioning & Localization Demo

System start:
- start UR16 
  - Programm: ROS2
  - Installation: FUS1
  - Boot/Enable robot



Launch nodes:

- On RPM-CU (192.168.1.2)

  - Start all services and drivers (don't execute code in tmux-session!):

    ```bash
    cd /home/bautiro/bautiro_ws/src/lu_waegele/launch/fus1
    ./eval_fus1_bringup_tmux.sh
    ```

  - Start Demo:
  
    ```bash
    ros2 run lu_fein_localization fein_loc_coordination_fus1
    ```

- On FPM-CU (192.168.2.2)
  - Start Driver:

    ```bash
    cd dev_ws
    source install/setup.bash
    ros2 launch fpm_ctrl_x_driver ctrl_x_driver.launch.py 
    ```

  - Move lift manually, input in meter **0<=height<2.4**

    ```bash  
    ros2 action send_goal /move_lift_absolute bautiro_ros_interfaces/action/MoveLiftAbsolute requested_target_lift_level:\ 0.0\
    ```

- On Spectra for Robot (192.168.2.7)
  - 1st shell:

    ```bash
    cd ros2_ws
    source install/setup.bash
    ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=192.168.2.3 launch_rviz:=false
    ```
    ```
    Start  the programm of the UR16
    ```


  - 2nd shell:

    ```bash 
    cd ros2_ws
    source install/setup.bash
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur16e launch_rviz:=false
    ```

  - 3rd shell:

    ```bash
    cd ros2_ws
    source install/setup.bash
    ros2 launch ur_moveit_config move_group_test.launch.py ur_type:=ur16e
    ```

  - Move robot manually to original pose:

    ```bash
    ros2 action send_goal /fpm_set_hu_configured_pose bautiro_ros_interfaces/action/SetHandlingUnitConfiguredPose handling_unit_configured_pose:\ 8\ 
    ```

  - Move robot manually to "Messpose":

    ```bash
    ros2 action send_goal /fpm_set_hu_configured_pose bautiro_ros_interfaces/action/SetHandlingUnitConfiguredPose handling_unit_configured_pose:\ 4\
    ```
    
For details see also [Docupedia - 2023-03-29 Demo Mai Positionierung und Lokalsierung "Fahrplan"](https://inside-docupedia.bosch.com/confluence/pages/viewpage.action?pageId=2899838260)
