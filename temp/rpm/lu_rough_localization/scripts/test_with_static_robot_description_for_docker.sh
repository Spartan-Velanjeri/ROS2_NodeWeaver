#! /bin/bash

WS_PATH=/home/$(whoami)/workspaces/galactic/develop/btr_ws

ROS_PATH=/opt/ros/galactic/setup.bash
WS=${WS_PATH}/install/setup.bash # modify this when working in local workspace

source ${ROS_PATH}
source ${WS}

# LOCAL_PKG="lu_rough_localization"
LU_ROUGH_LOCALIZATION_INSTALL_PATH="$(ros2 pkg prefix lu_rough_localization)/share/lu_rough_localization"
RPM_LOCALIZATION_INSTALL_PATH="$(ros2 pkg prefix rpm_localization)/share/rpm_localization"
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')  # replace "install" with "src"

SESSION=static_robot_descritption

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0

tmux select-window -t ${WINDOW}
tmux split-window -v
tmux split-window -v
tmux split-window -v

########## Resize panes
tmux select-pane -t ${WINDOW}.0
tmux resize-pane -U 10
tmux select-pane -t ${WINDOW}.1
tmux resize-pane -U 10
tmux select-pane -t ${WINDOW}.2
tmux resize-pane -U 15
tmux select-pane -t ${WINDOW}.3

tmux split-window -v

########## Exec commands

# Number are copied from
# rpm/src/rpm/rpm_description/config/RPM2/calibration_rpm.yaml

# rpm_lidar_fl: 
x=0.94340243 
y=-0.47297498 
z=0.60767897
roll=-0.0348805419
pitch=-0.00142631881 
yaw=-2.36216080
period_in_ms=100
frame_from=base_link
frame_to=rpm_lidar_front_lidar
tmux send-keys -t ${WINDOW}.0 "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${yaw} ${pitch} ${roll} ${frame_from} ${frame_to}" Enter

# rpm_lidar_rr:
x=-0.99641649 
y=0.46604675 
z=0.59825162
roll=0.00760024 
pitch=-0.00519648
yaw=-2.36199289
frame_from=base_link
frame_to=rpm_lidar_rear_lidar
tmux send-keys -t ${WINDOW}.1 "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${yaw} ${pitch} ${roll} ${frame_from} ${frame_to}" Enter


# rpm_imu:
x=0.72396978 
y=-0.35270053  
z=0.6527172
roll=0 
pitch=0
yaw=-1.570796327
frame_from=base_link
frame_to=rpm_imu
tmux send-keys -t ${WINDOW}.2 "source ${ROS_PATH} && ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${yaw} ${pitch} ${roll} ${frame_from} ${frame_to}" Enter


tmux send-keys -t ${WINDOW}.3 "source ${ROS_PATH} && ros2 run lu_rough_localization transform_services" Enter

MAP=${LU_ROUGH_LOCALIZATION_INSTALL_PATH}/maps/rosbag_le131_AUTOMATED_GEN
CONFIG_FILE_WITH_PATH=${RPM_LOCALIZATION_INSTALL_PATH}/config/RPM2/rpm_config.yaml
tmux send-keys -t ${WINDOW}.4 "source ${ROS_PATH}" Enter
# tmux send-keys -t ${WINDOW}.4 "echo ${LOCAL_PKG_INSTALL_PATH}" Enter
tmux send-keys -t ${WINDOW}.4 "ros2 launch rpm_localization rpm_localization_launch.py mode:=real hw_sample:=RPM2 map:=${MAP} config_file_with_path:=${CONFIG_FILE_WITH_PATH}"

########## Attach to the session
tmux select-pane -t ${WINDOW}.4
tmux attach-session -t "${SESSION}"