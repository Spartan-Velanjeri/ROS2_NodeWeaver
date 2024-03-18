#!/bin/bash

# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights

cd $(dirname "$(readlink -f "$0")")

source /opt/ros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash 2>/dev/null

dir=""
while true; do
    if source "${dir}install/setup.bash" 2>/dev/null; then
        break
    else
        dir="../${dir}"
        if [[ "$(realpath ${dir})" == "$(realpath /)" ]]; then
            echo "'install/setup.bash' - not found"
            exit 1
        fi
    fi
done

############################################################################
#
# sourcing done
#
############################################################################

current_active_mission() {
    echo '############################################################'
    echo '#'
    echo '# ros2 topic echo /current_active_mission --once'
    echo '#'
    echo '#'
    ros2 topic echo /current_active_mission_metadata --once
}

set_active_mission() {
    MISSION=$1
    echo '############################################################'
    echo '#'
    echo "#    /set_active_mission $MISSION"
    echo '#'
    ros2 service call /set_active_mission ccu_interfaces/srv/GenericPrim \
        "{data_bool: false, data_int: $MISSION, data_float: 0.0, data_str: '', proto: ''}"
}

unset_active_mission() {
    echo '############################################################'
    echo '#'
    echo "#    /unset_active_mission"
    echo '#'
    ros2 service call /unset_active_mission std_srvs/srv/Trigger
}


get_cluster_position() {
    echo '############################################################'
    echo '#'
    echo "#    /get_cluster_position"
    echo '#'
    ros2 service call /get_cluster_position bautiro_ros_interfaces/srv/GetClusterPosition
}
get_drill_holes_cluster() {
    echo '############################################################'
    echo '#'
    echo "#    /get_drill_holes_cluster"
    echo '#'
    ros2 service call /get_drill_holes_cluster bautiro_ros_interfaces/srv/GetDrillHolesCluster
}
get_markers_cluster() {
    echo '############################################################'
    echo '#'
    echo "#    /get_markers_cluster"
    echo '#'
    ros2 service call /get_markers_cluster bautiro_ros_interfaces/srv/GetMarkersCluster
}

#  defined now:
#
# current_active_mission
# set_active_mission
# unset_active_mission
# get_cluster_position
# get_drill_holes_cluster
# get_markers_cluster

unset_active_mission
set_active_mission 1
unset_active_mission
current_active_mission

get_cluster_position # Returns nothing

set_active_mission 1
current_active_mission

get_cluster_position

get_drill_holes_cluster

get_markers_cluster
