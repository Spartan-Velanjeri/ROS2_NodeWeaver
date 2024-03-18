#! /usr/bin/bash
# $1 position

USERNAME=bautiro
HOSTNAME=192.168.88.220
COMMAND='source /home/bautiro/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=200 && ros2 action send_goal /fpm_set_hu_configured_pose bautiro_ros_interfaces/action/SetHandlingUnitConfiguredPose "handling_unit_configured_pose: 4"'

# ssh ${USERNAME}@${HOSTNAME} 'bash -s' < test.sh true true true

echo $COMMAND
sshpass -p '100_bautiro_100' ssh -o StrictHostKeyChecking=no ${USERNAME}@${HOSTNAME} ${COMMAND}
#sshpass -p '100_bautiro_100' ssh -o StrictHostKeyChecking=no ${USERNAME}@${HOSTNAME} ". robot_goto_0.sh"
