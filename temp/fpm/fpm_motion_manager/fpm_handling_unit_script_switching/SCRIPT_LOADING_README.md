## Setup workspace

- Make sure that the packages `ros-galactic-ur-msgs` and `sudo apt purge ros-galactic-ur-description` are not installed as binaries.

Checkout following needed repositories:
```
repositories:
  Universal_Robots_ROS2_Driver:
    type: git
    url: https://github.com/StoglRobotics-forks/Universal_Robots_ROS2_Driver.git
    version: galactic-script-loading
  Universal_Robots_ROS2_Description:
    type: git
    url: https://github.com/StoglRobotics-forks/Universal_Robots_ROS2_Description.git
    version: galactic-script-loading
  moveit2:
    type: git
    url: https://github.com/StoglRobotics-forks/moveit2.git
    version: galactic-remove-project-method
  ur_msgs:
    type: git
    url: https://github.com/StoglRobotics-forks/ur_msgs.git
    version: aux-scripts-msgs
```

## Before starting

- Setup [custom_script1.urscript](./ur_robot_driver/resources/custom_script_1.urscript):

  - Setup up address of computer running driver - `COMPUTER_DRIVER_IP` variable
    (if using `ursim` in docker then: "192.168.56.1")

- Setup parameters for hardware interface in [robot description for using custom scripts](../Universal_Robots_ROS2_Description/urdf/ur.ros2_control.xacro):

  - `aux_script_base_path` - folder where the scripts are stored
  - `aux_script_filenames` - names of the script(s) separated with `;`
  - `aux_script_arguments` - names of arguments to replace with the script separated with `;`, and written without leading `<<` and trailling `>>` placeholder markers

- Setup script arguments names on the controller side in (controllers.yaml file)[ur_bringup/config/ur_controllers.yaml] under `io_and_status_controller`:

  - Adjust the list under `aux_script_arguments` - the list has to have the same arguments as `aux_script_arguments` in the xacro file.


## Starting

1. Start robot or [`ursim`](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)

2. Start driver: ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=<ROBOT_IP> headless_mode:=true launch_rviz:=false

   - For `ursim`: ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=192.168.56.101 headless_mode:=true launch_rviz:=false

3. Start node for shared variables:
   ```
   ros2 launch ur_bringup monitoring_node.launch.py robot_ip:=<ROBOT_IP>
   ```

4. Control robot using MoveIt2 and ros2_control

5. Set arguments in the script using dedicated service:

   - Version 1 - using `all_arguments` filed (useful for development because you don't need to extend messages when adding new arguments)
     ```
     ros2 service call /io_and_status_controller/set_aux_script_arguments ur_msgs/srv/SetAuxiliaryScriptArguments "
     all_arguments: [-0.2, -0.55, -0.20, 0.05]
     drill_pos_x_replace: 0.0
     drill_pos_y_replace: 0.0
     drill_pos_z_replace: 0.0
     drill_depth_replace: 0.0"
     ```

   - Version 2 - using dedicated fields (useful for production because we know explicitly what is added where - extend `ur_msgs/srv/SetAuxiliaryScriptArguments.srv` message and controller with additional fields)
     ```
     ros2 service call /io_and_status_controller/set_aux_script_arguments ur_msgs/srv/SetAuxiliaryScriptArguments "
     all_arguments: []
     drill_pos_x_replace: -0.2
     drill_pos_y_replace: -0.55
     drill_pos_z_replace: -0.20
     drill_depth_replace: 0.1"
     ```

5. Switch script using - robots starts to execute cyclic movements
   ```
   ros2 service call /io_and_status_controller/switch_script std_srvs/srv/Trigger {}
   ```

6. Setup movement distance using montoring node:
   ```
   ros2 topic pub /monitoring_node/var0 std_msgs/msg/Float64 "{data: 0.15}"
   ```

7. Stop/Terminate script execution:
   ```
   ros2 service call /monitoring_node/stop_main_loop std_srvs/srv/Trigger {}
   ```

8. Switch again `ros2_control` script to use MoveIt2/ros2_control:
   ```
   ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger {}
   ```
