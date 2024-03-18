#! /bin/bash


ROS_PATH=/opt/ros/galactic/setup.bash
WS_EXTERNAL_PATH=/opt/bautiro/btr_external_ws/install/setup.bash
WS_PATH=/home/$(whoami)/workspace/workspaces/galactic/default/btr_ws/
WS=${WS_PATH}/install/setup.bash

source ${ROS_PATH}
source ${WS_EXTERNAL_PATH}
source ${WS}

SESSION=fpm1_bringup_tmux

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0

WINDOW_FPM=${SESSION}:1

#Create window
tmux new-window -t ${WINDOW_FPM} -n FPM


#### Start FPM ##### 
tmux select-window -t ${WINDOW_FPM}
tmux split-window -h #0.1
tmux select-pane -t ${WINDOW_FPM}.0
tmux split-window -v #0.2
tmux split-window -v #0.2
tmux select-pane -t ${WINDOW_FPM}.3
tmux split-window -v #1.3
tmux split-window -v #1.3

#tmux send-keys -t ${WINDOW_FPM}.0 C-z "source ${WS} && ros2 launch fpm_moveit ur_moveit.launch.py launch_rviz:=false" #Enter
#tmux send-keys -t ${WINDOW_FPM}.1 C-z "source ${WS} && ros2 launch handling_unit_motion_manager main.launch.py" #Enter
#tmux send-keys -t ${WINDOW_FPM}.2 C-z "source ${WS} && ros2 launch hu_behavior_tree hu_tree_bringup.launch.py" #Enter
tmux send-keys -t ${WINDOW_FPM}.3 C-z "source ${ROS_PATH} && ros2 launch fpm_sensors fpm_sensors_launch.py mode:=real hw_sample:=FPM1 config_file_with_path:=${WS_PATH}/fpm/install/fpm_sensors/share/fpm_sensors/config/FPM1/fpm_config.yaml" Enter
tmux send-keys -t ${WINDOW_FPM}.4 C-z "source ${ROS_PATH} && ros2 launch fpm_localization fpm_localization_launch.py" Enter
tmux send-keys -t ${WINDOW_FPM}.5 C-z "source ${ROS_PATH} && ros2 launch fpm_bringup fpm.launch.py hw_version:='FUS1'" Enter

########## Attach to the session
tmux select-pane -t ${WINDOW_FPM}.0
tmux attach-session -t "${SESSION}"
