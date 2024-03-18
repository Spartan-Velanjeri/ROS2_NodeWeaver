#! /bin/bash


ROS_PATH=/opt/ros/galactic/setup.bash
WS=/bautiro_environment/bautiro_setup.bash # modify this when working in local workspace

source ${ROS_PATH}
source ${WS}

# LOCAL_PKG="bautiro"
# LOCAL_PKG_INSTALL_PATH=$(ros2 pkg prefix ${LOCAL_PKG})
# LOCAL_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${LOCAL_PKG} | sed --expression='s/install/src/g')

SESSION=rpm_bringup

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0
# pane=${WINDOW}.4

# Create panes
tmux split-window -h
tmux select-pane -t ${WINDOW}.0
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-pane -t ${WINDOW}.4
tmux split-window -v
tmux split-window -v
tmux split-window -v


# Send commands
# CMD_PATH="bash ${LOCAL_PKG_WORKSPC_PATH}/launch/fus1/"
tmux send-keys -t ${WINDOW}.0 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch bautiro_bringup bautiro_description.launch.py" Enter
tmux send-keys -t ${WINDOW}.1 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_sensors rpm_sensors_launch.py mode:=real hw_sample:=RPM1 config_file_with_path:=/bautiro_environment/bautiro_core_develop/rpm/install/rpm_sensors/share/rpm_sensors/config/RPM1/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW}.2 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_localization rpm_localization_launch.py mode:=real hw_sample:=RPM1 map:=/bautiro_environment/bautiro_core_develop/rpm/src/lu_rough_localization/maps/rosbag_shroom1_externed_cropped config_file_with_path:=/bautiro_environment/bautiro_core_develop/rpm/install/rpm_localization/share/rpm_localization/config/RPM1/rpm_config.yaml" Enter
tmux send-keys -t ${WINDOW}.3 C-z "source ${ROS_PATH} && source /home/bautiro/bautiro_ws/install/setup.bash; ros2 run lu_fein_localization tf2_reflex_marker_broadcaster" Enter

# Command for controller and navigation
# activate CAN-Mode for RPM
#bash /bautiro_environment/bautiro_core_develop/rpm/src/rpm/rpm_actuators/scripts/RPM_CAN_Mode_activation
# activate ros diff driver controller 
#ros2 service call /rpm/left_wheel_controller/nmt_start_node std_srvs/srv/Trigger {}
# teleop with keyboard
#ros2 run teleop_twist_keyboard teleop_twist_keyboard --remap /cmd_vel:=/rpm/diff_base_controller/cmd_vel_unstamped

tmux send-keys -t ${WINDOW}.4 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_control rpm_diffdrive_controller_launch.py robot_name:=FUS1 rpm_name:=RPM1" Enter
tmux send-keys -t ${WINDOW}.5 C-z "source ${ROS_PATH} && source ${WS} && ros2 run tf2_ros static_transform_publisher 0.0 0.0 -3.84 0.0 0.0 0.0 map map_floor" Enter

tmux send-keys -t ${WINDOW}.6 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_nav2 navigation_launch.py" #Enter
tmux send-keys -t ${WINDOW}.7 C-z "source ${ROS_PATH} && source ${WS} && ros2 launch rpm_nav2 map_launch.py" #Enter

# Attach to the session
tmux select-pane -t ${WINDOW}.0
# tmux select-window -t "$WINDOW"
tmux attach-session -t "${SESSION}"