#! /bin/bash
ROS_PATH=/opt/ros/galactic/setup.bash
WS=/bautiro_environment/bautiro_setup.bash # modify this when working in local workspace

source ${ROS_PATH}
source ${WS}

FPM_SENSORS_PKG="fpm_sensors"
FPM_SENSORS_PKG_INSTALL_PATH="$(ros2 pkg prefix ${FPM_SENSORS_PKG})/share/fpm_sensors"
FPM_SENSORS_PKG_WORKSPC_PATH=$(ros2 pkg prefix ${FPM_SENSORS_PKG} | sed --expression='s/install/src/g')

SESSION=test_env_fine_localization_service
# SESSION=${BASH_SOURCE[${#BASH_SOURCE[@]} - 1]}

tmux kill-session -t ${SESSION}
tmux new-session -s ${SESSION} -d

WINDOW=${SESSION}:0
# pane=${WINDOW}.4

# Create panes
tmux split-window -h
tmux select-pane -t ${WINDOW}.0
tmux split-window -v
tmux split-window -v
tmux select-pane -t ${WINDOW}.3
tmux split-window -v
tmux split-window -v

# Example #1:
# Room size 2x2m
# Origin (0) in the lower left corner and 3 markers
#      ---------
#      |       |
#     (2)     (3)
#      |       |
#      0--(1)---
#
cmd_call_srv='ros2 service call /fine_localization lu_fine_localization/srv/LuFineLocalization "
{ markers: 
    [{
        marker_id: 'm1',
        pose: {pose: {position: {x: 1, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: 0.0, y: 1.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: 2.0, y: 1.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    }]
}
"'

SIMULATED_POSE="1.0 1.0 0.5 1.7 0.0 0.0" # (x,y,z,yaw,pitch,roll)

# Send commands
# CMD_PATH="bash ${LOCAL_PKG_WORKSPC_PATH}/launch/fus1/"
tmux send-keys -t ${WINDOW}.0 "ros2 launch fpm_sensors fpm_sensors_launch.py mode:=gazebo_sim hw_sample:=FPM1 config_file_with_path:=${FPM_SENSORS_PKG_INSTALL_PATH}/config/FPM1/fpm_config.yaml" Enter
tmux send-keys -t ${WINDOW}.1 "ros2 run lu_rough_localization transform_services" Enter
tmux send-keys -t ${WINDOW}.2 "${cmd_call_srv}"
tmux send-keys -t ${WINDOW}.3 "ros2 run tf2_ros static_transform_publisher ${SIMULATED_POSE} map fpm_totalstation" Enter
tmux send-keys -t ${WINDOW}.4 "ros2 launch fpm_localization fpm_localization_launch.py" Enter
# tmux send-keys -t ${WINDOW}.4 "ros2 run lu_fine_localization fine_localization_service.py" Enter


# Attach to the session
tmux select-pane -t ${WINDOW}.2
# tmux select-window -t "$WINDOW"
tmux attach-session -t "${SESSION}"
