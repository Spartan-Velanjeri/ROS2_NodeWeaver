#! /bin/bash


ROS_PATH=/opt/ros/galactic/setup.bash
WS=/bautiro_environment/bautiro_setup.bash # modify this when working in local workspace

source ${ROS_PATH}
source ${WS}

# LOCAL_PKG="bautiro"
# LOCAL_PKG_INSTALL_PATH=$(ros2 pkg prefix ${LOCAL_PKG})
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')

SESSION=sim_bringup

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW_CCU=${SESSION}:1
WINDOW_RPM=${SESSION}:2
WINDOW_FPM=${SESSION}:3
WINDOW_HCU=${SESSION}:4
WINDOW_MISC=${SESSION}:5

#Create window
tmux new-window -t ${WINDOW_CCU} -n CCU 
tmux new-window -t ${WINDOW_RPM} -n RPM
tmux new-window -t ${WINDOW_FPM} -n FPM
tmux new-window -t ${WINDOW_HCU} -n HCU 
tmux new-window -t ${WINDOW_MISC} -n MISC


##### Start HCU  ####### 
tmux select-window -t ${WINDOW_HCU}
tmux split-window -h
## ToDo Command To start HCU on Ubuntu machine




##### Start CCU   #####
tmux select-window -t ${WINDOW_CCU}
tmux send-keys -t ${WINDOW_CCU}.0 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch bautiro_gazebo_simulation bautiro_spawn_gazebo_control.launch.py" Enter
tmux split-window -h #0.1
### others command to start CCU
tmux split-window -v


##### Start RPM #####
tmux select-window -t ${WINDOW_RPM}
tmux split-window -h #0.1
tmux select-pane -t ${WINDOW_RPM}.0
tmux split-window -v #0.2
tmux select-pane -t ${WINDOW_RPM}.1
tmux split-window -v #1.3

tmux send-keys -t ${WINDOW_RPM}.0 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_localization rpm_localization_launch.py mode:=gazebo_sim hw_sample:=RPM1 map:=/bautiro_environment/bautiro_core_develop/rpm/src/lu_rough_localization/maps/rosbag_le131_AUTOMATED_GEN  config_file_with_path:=/bautiro_environment/bautiro_core_develop/rpm/install/rpm_localization/share/rpm_localization/config/RPM1/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW_RPM}.1 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_nav2 navigation_launch.py use_sim_time:=true" Enter
tmux send-keys -t ${WINDOW_RPM}.2 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_nav2 map_launch.py" Enter
tmux send-keys -t ${WINDOW_RPM}.3 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_behavior_tree rpm_tree_bringup.launch.py" Enter


#### Start FPM ##### 
tmux select-window -t ${WINDOW_FPM}
tmux split-window -h #0.1
tmux select-pane -t ${WINDOW_FPM}.0
tmux split-window -v #0.2
tmux select-pane -t ${WINDOW_FPM}.1
tmux split-window -v #1.3

tmux send-keys -t ${WINDOW_FPM}.0 C-z "source ${WS} && ros2 launch fpm_moveit ur_moveit.launch.py launch_rviz:=false" Enter
tmux send-keys -t ${WINDOW_FPM}.1 C-z "source ${WS} && ros2 launch handling_unit_motion_manager main.launch.py" Enter
tmux send-keys -t ${WINDOW_FPM}.2 C-z "source ${WS} && ros2 launch hu_behavior_tree hu_tree_bringup.launch.py" Enter


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



########## Attach to the session
tmux attach-session -t "${SESSION}"
tmux select-pane -t ${WINDOW_CCU}.0
