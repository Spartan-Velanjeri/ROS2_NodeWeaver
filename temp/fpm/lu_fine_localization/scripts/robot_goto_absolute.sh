#! /usr/bin/bash
# $1 position

USERNAME=bautiro
HOSTNAME=192.168.88.220

COMMAND="source /home/bautiro/ros2_ws/install/setup.bash; ros2 action send_goal /fpm_set_move_hu_absolute bautiro_ros_interfaces/action/MoveHandlingUnitAbsolute \"absolute_target_position: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"\"}, pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}}\""

# COMMAND="ros2 topic pub --once  /test geometry_msgs/PoseStamped \"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"world\"}, pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}} \""


# header:\n
#     stamp:\n
#       sec: 0\n
#       nanosec: 0\n
#     frame_id: ''\n
#   pose:\n
#     position:\n
#       x: '$1'\n
#       y: '$2'\n
#       z: '$3'\n
#     orientation:\n
#       x: '$4'\n
#       y: '$5'\n
#       z: '$6'\n
#       w: '$7'"'

echo ""
echo ${COMMAND}
# printf "%s" ${COMMAND}
echo ""

# eval ${COMMAND}

sshpass -p '100_bautiro_100' ssh -o StrictHostKeyChecking=no ${USERNAME}@${HOSTNAME} "eval ${COMMAND}"
