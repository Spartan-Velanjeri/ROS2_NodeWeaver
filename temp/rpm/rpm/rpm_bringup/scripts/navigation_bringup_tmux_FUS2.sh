#! /bin/bash
ROS_PATH=/opt/ros/galactic/setup.bash
WS=/bautiro_environment/842/setup.bash # modify this when working in local workspace

source ${ROS_PATH}
source ${WS}

# LOCAL_PKG="bautiro"
# LOCAL_PKG_INSTALL_PATH=$(ros2 pkg prefix ${LOCAL_PKG})
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')

SESSION=rpm_bringup_navigation

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0

WINDOW_CCU=${SESSION}:1
WINDOW_RPM=${SESSION}:2
WINDOW_RPM2=${SESSION}:3
WINDOW_FPM=${SESSION}:4
WINDOW_HCU=${SESSION}:5
WINDOW_MISC=${SESSION}:6

#Create window
tmux new-window -t ${WINDOW_CCU} -n CCU 
tmux new-window -t ${WINDOW_RPM} -n RPM
tmux new-window -t ${WINDOW_RPM2} -n RPM2
tmux new-window -t ${WINDOW_FPM} -n FPM
tmux new-window -t ${WINDOW_HCU} -n HCU 
tmux new-window -t ${WINDOW_MISC} -n MISC


##### Start HCU  ####### 
tmux select-window -t ${WINDOW_HCU}
tmux split-window -h
## ToDo Command To start HCU on Ubuntu machine

##### Start CCU   #####
tmux select-window -t ${WINDOW_CCU}

# # launch decription + simulation
# tmux send-keys -t ${WINDOW_CCU}.0 C-z "source ${ROS_PATH} && ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py" Enter

# launch only description for hardware
tmux send-keys -t ${WINDOW_CCU}.0 C-z "source ${ROS_PATH} && ros2 launch bautiro_bringup bautiro_description.launch.py robot_name:=FUS2 rpm_name:=RPM2" Enter

tmux split-window -h #0.1
### others command to start CCU
tmux split-window -v


##### Start RPM (part1) #####
tmux select-window -t ${WINDOW_RPM}
tmux split-window -h
tmux select-pane -t ${WINDOW_RPM}.0
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-pane -t ${WINDOW_RPM}.4
tmux split-window -v
tmux split-window -v
tmux split-window -v

# put command to panes
tmux send-keys -t ${WINDOW_RPM}.0 C-z "source ${ROS_PATH} && ros2 launch rpm_sensors rpm_sensors_launch.py mode:=real hw_sample:=RPM2 config_file_with_path:=/bautiro_environment/842/rpm/install/rpm_sensors/share/rpm_sensors/config/RPM2/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW_RPM}.1 C-z "source ${ROS_PATH} && ros2 launch rpm_localization rpm_localization_launch.py mode:=real hw_sample:=RPM2 map:=/bautiro_environment/842/rpm/install/lu_rough_localization/share/lu_rough_localization/maps/rosbag_le131_AUTOMATED_GEN config_file_with_path:=/bautiro_environment/842/rpm/install/rpm_localization/share/rpm_localization/config/RPM2/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW_RPM}.2 C-z "source ${ROS_PATH} && ros2 run lu_rough_localization filter_temporal_spatial_outliers --ros-args -p tpc_in:=/rpm/sensors/front/lidar3d/points -p max_distance:=5.0" Enter
tmux send-keys -t ${WINDOW_RPM}.3 C-z "source ${ROS_PATH} && ros2 run lu_rough_localization filter_temporal_spatial_outliers --ros-args -p tpc_in:=/rpm/sensors/rear/lidar3d/points -p max_distance:=5.0" Enter
tmux send-keys -t ${WINDOW_RPM}.4 C-z "source ${ROS_PATH}" Enter
tmux send-keys -t ${WINDOW_RPM}.5 C-z "source ${ROS_PATH} && ros2 launch rpm_control rpm_diffdrive_controller_launch.py robot_name:=FUS2 rpm_name:=RPM2" Enter
tmux send-keys -t ${WINDOW_RPM}.6 C-z "source ${ROS_PATH} && ros2 run p2p_offset_controller controller" Enter


##### Start RPM (part2) #####
tmux select-window -t ${WINDOW_RPM2}
tmux split-window -h
tmux select-pane -t ${WINDOW_RPM2}.0
tmux split-window -v
tmux select-pane -t ${WINDOW_RPM2}.2
tmux split-window -v

# put command to panes
tmux send-keys -t ${WINDOW_RPM2}.0 C-z "source ${ROS_PATH} && ros2 launch rpm_nav2 navigation_launch.py velocity_command_topic:=/rpm/diff_base_controller/cmd_vel_unstamped" #Enter
tmux send-keys -t ${WINDOW_RPM2}.2 C-z "source ${ROS_PATH} && ros2 launch rpm_behavior_tree rpm_tree_bringup.launch.py" Enter
tmux send-keys -t ${WINDOW_RPM2}.1 C-z "source ${ROS_PATH} && ros2 launch rpm_nav2 map_launch.py" #Enter


#### Start FPM ##### 
tmux select-window -t ${WINDOW_FPM}
tmux split-window -h #0.1
tmux select-pane -t ${WINDOW_FPM}.0
tmux split-window -v #0.2
tmux select-pane -t ${WINDOW_FPM}.1
tmux split-window -v #1.3

tmux send-keys -t ${WINDOW_FPM}.0 C-z "source ${ROS_PATH} && ros2 launch fpm_moveit ur_moveit.launch.py launch_rviz:=false" #Enter
tmux send-keys -t ${WINDOW_FPM}.1 C-z "source ${ROS_PATH} && ros2 launch handling_unit_motion_manager main.launch.py" #Enter
tmux send-keys -t ${WINDOW_FPM}.2 C-z "source ${ROS_PATH} && ros2 launch hu_behavior_tree hu_tree_bringup.launch.py" #Enter


### Start some other functions ####
tmux select-window -t ${WINDOW_MISC}
tmux select-pane -t ${WINDOW_MISC}.0
tmux split-window -v #0.1
tmux split-window -v #1.2
tmux select-pane -t ${WINDOW_MISC}.0
tmux split-window -v #0.3
tmux select-pane -t ${WINDOW_MISC}.1
tmux split-window -v #1.4
tmux select-pane -t ${WINDOW_MISC}.2
tmux split-window -v #2.5

tmux send-keys -t ${WINDOW_MISC}.0 C-z "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher -4.742 3.396 0.0 0.0 0.0 0.0 map cluster1" Enter
tmux send-keys -t ${WINDOW_MISC}.1 C-z "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher -0.742 3.396 0.0 0.0 0.0 0.0 map cluster2" Enter


########## Attach to the session
tmux attach-session -t "${SESSION}"
tmux select-pane -t ${WINDOW_CCU}.0




###### Some command for copy paste

#tmux send-keys -t ${WINDOW}.3 C-z "source ${ROS_PATH} && ros2 launch rpm_control rpm_diffdrive_controller_orig_launch.py robot_name:=FUS2 rpm_name:=RPM2" Enter
# ros2 run p2p_offset_controller controller

# tmux send-keys -t ${WINDOW}.1 C-z "source /opt/ros/galactic/setup.bash;source /bautiro_environment/bautiro_setup_305.bash;ros2 launch rpm_control rpm_diffdrive_controller_launch.py" Enter
# tmux send-keys -t ${WINDOW}.3 C-z "source /opt/ros/galactic/setup.bash;source /bautiro_environment/bautiro_setup_305.bash;ros2 launch rpm_nav2 navigation_launch.py" Enter
# tmux send-keys -t ${WINDOW}.4 C-z "source /opt/ros/galactic/setup.bash;source /bautiro_environment/bautiro_setup_305.bash;ros2 launch rpm_nav2 map_launch.py" Enter
# Command for controller and navigation
# activate CAN-Mode for RPM
#bash /bautiro_environment/428_bautiro_core/rpm/src/rpm/rpm_actuators/scripts/RPM_CAN_Mode_activation
# activate ros diff driver controller 
#ros2 service call /rpm/left_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}
#ros2 service call /rpm/right_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}

# teleop with keyboard
#ros2 run teleop_twist_keyboard teleop_twist_keyboard --remap /cmd_vel:=/rpm/diff_base_controller/cmd_vel_unstamped

# start alll
#bash /bautiro_environment/428_bautiro_core/rpm/src/rpm/rpm_bringup/scripts/navigation_bringup_tmux_FUS2.sh

# clear costmap
# ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "request: {}"
# ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "request: {}"

# Start nav2 navigate_to_pose
# ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
#   header:
#     stamp:
#       sec: 0
#       nanosec: 0
#     frame_id: ''
#   pose:
#     position:
#       x: -10.0
#       y: 0.918
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
# behavior_tree: ''"

# Start p2p
# ros2 action send_goal /rpm_start bautiro_ros_interfaces/action/StartRpmMainTask "start: true 
# bt_xml: ''
# des_cluster_pose:
#   cluster_id: ''
#   pks_pose:
#     position:
#       x: -6.482
#       y: 0.896
#       z: 0.179
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
# reference_frame: 'map'" 

# Start Nav2
# ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
#   header:
#     stamp:
#       sec: 0
#       nanosec: 0
#     frame_id: 'map'
#   pose:
#     position:,
#       x: 1.6
#       y: -0.08 #2.0 
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
# behavior_tree: ''"

# Start P2P
# ros2 action send_goal /rpm_start bautiro_ros_interfaces/action/StartRpmMainTask "start: true
# bt_xml: ''
# des_cluster_pose:
#   cluster_id: ''
#   pks_pose:
#     position:
#       x: 5.152 
#       y: -0.138
#       z: 0.147
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
# reference_frame: 'map'"