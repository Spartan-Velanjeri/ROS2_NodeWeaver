#! /bin/bash

ROS_PATH=/opt/ros/galactic/setup.bash
WS_EXTERNAL_PATH=/opt/bautiro/btr_external_ws/install/setup.bash
WS_PATH=/home/$(whoami)/workspaces/galactic/develop/btr_ws
WS=${WS_PATH}/install/setup.bash

source ${ROS_PATH}
source ${WS_EXTERNAL_PATH}
source ${WS}

SESSION=rpm1_bringup_tmux

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0

WINDOW_CCU=${SESSION}:1
WINDOW_RPM_1OF2=${SESSION}:2
WINDOW_RPM_2OF2=${SESSION}:3

#Create window
tmux new-window -t ${WINDOW_CCU} -n CCU 
tmux new-window -t ${WINDOW_RPM_1OF2} -n RPM_1of2
tmux new-window -t ${WINDOW_RPM_2OF2} -n RPM_2of2

##### Start CCU   #####
tmux select-window -t ${WINDOW_CCU}

# launch only description for hardware
tmux send-keys -t ${WINDOW_CCU}.0 "source ${ROS_PATH} && ros2 launch bautiro_description robot.launch.py robot_name:=FUS1 rpm_name:=RPM1" Enter

tmux split-window -h #0.1
### others command to start CCU
tmux split-window -v


##### Start RPM #####
tmux select-window -t ${WINDOW_RPM_1OF2}
tmux split-window -h
tmux select-pane -t ${WINDOW_RPM_1OF2}.0
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-pane -t ${WINDOW_RPM_1OF2}.4
tmux split-window -v
tmux split-window -v
tmux split-window -v

# put command to panes
tmux send-keys -t ${WINDOW_RPM_1OF2}.0 "source ${ROS_PATH} && ros2 launch rpm_sensors rpm_sensors_launch.py mode:=real hw_sample:=RPM1 config_file_with_path:=$(ros2 pkg prefix --share rpm_sensors)/config/RPM1/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.1 "source ${ROS_PATH} && ros2 launch rpm_localization rpm_localization_launch.py mode:=real hw_sample:=RPM1 map:=$(ros2 pkg prefix --share lu_rough_localization)/maps/rosbag_le131" Enter
# tmux send-keys -t ${WINDOW_RPM_1OF2}.1 "source ${ROS_PATH} && ros2 launch rpm_localization rpm_localization_launch.py mode:=real hw_sample:=RPM1 map:=$(ros2 pkg prefix --share lu_rough_localization)/maps/rosbag_le131_AUTOMATED_GEN" Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.2 "source ${ROS_PATH} && ros2 run lu_rough_localization filter_temporal_spatial_outliers --ros-args -p tpc_in:=/rpm/sensors/front/lidar3d/points -p max_distance:=5.0" Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.3 "source ${ROS_PATH} && ros2 run lu_rough_localization filter_temporal_spatial_outliers --ros-args -p tpc_in:=/rpm/sensors/rear/lidar3d/points -p max_distance:=5.0" Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.4 "bash $BTR_WS/workspaces/galactic/develop/btr_ws/src/rpm/rpm/rpm_actuators/scripts/RPM_CAN_Mode_activation" #Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.5 "source ${ROS_PATH} && ros2 launch rpm_control rpm_diffdrive_controller_launch.py robot_name:=FUS2 rpm_name:=RPM2" Enter
#tmux send-keys -t ${WINDOW_RPM_1OF2}.6 "source ${ROS_PATH} && ros2 run p2p_offset_controller controller" Enter
tmux send-keys -t ${WINDOW_RPM_1OF2}.7 "source ${ROS_PATH} && ros2 service call /rpm/left_wheel_controller/nmt_start_node std_srvs/srv/Trigger {} && sleep 0.5 && ros2 service call /rpm/right_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}" #Enter

##### Start RPM (part2) #####
tmux select-window -t ${WINDOW_RPM_2OF2}
tmux split-window -h
tmux select-pane -t ${WINDOW_RPM_2OF2}.0
tmux split-window -v
tmux select-pane -t ${WINDOW_RPM_2OF2}.2
tmux split-window -v
tmux send-keys -t ${WINDOW_RPM_2OF2}.0 "source ${ROS_PATH} && ros2 launch rpm_nav2 navigation_launch.py velocity_command_topic:=/rpm/diff_base_controller/cmd_vel_unstamped" #Enter
tmux send-keys -t ${WINDOW_RPM_2OF2}.1 "source ${ROS_PATH} && ros2 launch rpm_nav2 map_launch.py" #Enter
tmux send-keys -t ${WINDOW_RPM_2OF2}.2 "source ${ROS_PATH} && ros2 launch rpm_behavior_tree rpm_tree_bringup.launch.py" Enter
tmux send-keys -t ${WINDOW_RPM_2OF2}.3 "source ${ROS_PATH} && ros2 run teleop_twist_keyboard teleop_twist_keyboard --remap /cmd_vel:=/rpm/diff_base_controller/cmd_vel_unstamped" #Enter



########## Attach to the session
tmux select-pane -t ${WINDOW_CCU}.0
tmux attach-session -t "${SESSION}"


# Commands for enabling drive control
# Press NOT-AUS !!!
# bash btr_ws/src/rpm/rpm/rpm_actuators/scripts/RPM_CAN_Mode_activation
# ros2 service call /rpm/left_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}
# ros2 service call /rpm/right_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}
