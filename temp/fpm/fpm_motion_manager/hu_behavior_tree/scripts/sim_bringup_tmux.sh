#! /bin/bash

ROS_PATH=/opt/ros/galactic/setup.bash
WS=~/dev_ws/install/setup.bash # modify this when working in local workspace
WS_SRC=~/dev_ws/src
source ${ROS_PATH}
source ${WS}

# LOCAL_PKG="bautiro"
# LOCAL_PKG_INSTALL_PATH=$(ros2 pkg prefix ${LOCAL_PKG})
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')

SESSION=sim_bringup

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0
# pane=${WINDOW}.4

# Create panes
tmux split-window -h
tmux split-window -h
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-layout tiled


# Send commands
# CMD_PATH="bash ${LOCAL_PKG_WORKSPC_PATH}/launch/fus1/"
tmux send-keys -t ${WINDOW}.0 C-z "cd && ursim-5.12.4.1101661/start-ursim.sh UR16e" Enter
tmux send-keys -t ${WINDOW}.1 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch hu_behavior_tree hu_tree_bringup.launch.py" Enter
tmux send-keys -t ${WINDOW}.2 C-z "source ${ROS_PATH} && source ${WS} && sleep 15 && ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=10.30.36.200 launch_rviz:=false headless_mode:=true" Enter
tmux send-keys -t ${WINDOW}.3 C-z "source ${ROS_PATH} && source ${WS} && sleep 25 && ros2 launch bautiro_bringup bautiro_bringup.launch.py" Enter
tmux send-keys -t ${WINDOW}.4 C-z "source ${ROS_PATH} && source ${WS} && ros2 run groot Groot --mode monitor --publisher_port 2200 --server_port 2201" Enter
tmux send-keys -t ${WINDOW}.5 C-z "source ${ROS_PATH} && source ${WS} && ros2 action send_goal /hu_start bautiro_ros_interfaces/action/StartHuMainTask "{start: true, bt_xml: "moveit_program_control.xml"}""
tmux set-option -g mouse on
# Attach to the session
tmux select-pane -t ${WINDOW}.5
# tmux select-window -t "$WINDOW"
tmux attach-session -t "${SESSION}"