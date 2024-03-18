#! /bin/bash
ROS_PATH=/opt/ros/galactic/setup.bash
WS_EXTERNAL_PATH=/opt/bautiro/btr_external_ws/install/setup.bash
WS_PATH=/home/$(whoami)/workspace/workspaces/galactic/develop/btr_ws
WS=${WS_PATH}/install/setup.bash

source ${ROS_PATH}
source ${WS_EXTERNAL_PATH}
source ${WS}

# LOCAL_PKG="lu_rough_localization"
# LU_ROUGH_LOCALIZATION_INSTALL_PATH="$(ros2 pkg prefix lu_rough_localization)/share/lu_rough_localization"
# RPM_LOCALIZATION_INSTALL_PATH="$(ros2 pkg prefix rpm_localization)/share/rpm_localization"
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')  # replace "install" with "src"

SESSION=static_robot_descritption

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0

tmux select-window -t ${WINDOW}
tmux split-window -v
# tmux split-window -v
# tmux split-window -v

########## Resize panes
tmux select-pane -t ${WINDOW}.0
# tmux resize-pane -U 10
tmux select-pane -t ${WINDOW}.1
# tmux resize-pane -U 10
# tmux select-pane -t ${WINDOW}.2
# tmux resize-pane -U 15
# tmux select-pane -t ${WINDOW}.3

tmux split-window -v

########## Exec commands

# Number are copied from
# rpm/src/rpm/rpm_description/config/RPM2/calibration_rpm.yaml

# rpm_lidar_fl: 
x=0.0
y=0.0
z=0.0
roll=0.0
pitch=0.0
yaw=1.57
period_in_ms=100
frame_from=rpm_lidar_front
frame_to=rpm_lidar_front_lidar
tmux send-keys -t ${WINDOW}.0 C-z "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${yaw} ${pitch} ${roll} ${frame_from} ${frame_to}" Enter

# rpm_lidar_rr:
x=0.0
y=0.0
z=0.0
roll=0.0
pitch=0.0
yaw=1.57
frame_from=rpm_lidar_rear
frame_to=rpm_lidar_rear_lidar
tmux send-keys -t ${WINDOW}.1 C-z "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${yaw} ${pitch} ${roll} ${frame_from} ${frame_to}" Enter


########## Attach to the session
tmux select-pane -t ${WINDOW}.3
# tmux attach-session -t "${SESSION}"
echo "Section  ${SESSION} started in background."