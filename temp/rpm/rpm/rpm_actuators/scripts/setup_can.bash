#!/bin/bash

# Setup CAN configuration for using ros2_canopen
#
# Usage: ./setup_can.bash "can interface name" [bitrate]
#
#
# Authors: Dr. Denis (Stogl Robotics Consulting)


CAN_INTERFACE_NAME=$1
if [ -z "${CAN_INTERFACE_NAME}" ]; then
  CAN_INTERFACE_NAME=can0
  echo "Can interface name not set 🤔. Using default '${CAN_INTERFACE_NAME}'!"
fi

BITRATE=$2
if [ -z "${BITRATE}" ]; then
  BITRATE=250000
  echo "Bitrate is not set 🤔. Using default '1M' (${BITRATE})!"
fi

if [[ "${CAN_INTERFACE_NAME}" == "vcan"* ]]; then
  echo "Initializing 'vcan' module"
  sudo modprobe can
  sudo ip link add dev ${CAN_INTERFACE_NAME} type can
else
  sudo ip link set ${CAN_INTERFACE_NAME} up type can bitrate ${BITRATE}
fi

sudo ip link set ${CAN_INTERFACE_NAME} txqueuelen 1000
sudo ip link set up ${CAN_INTERFACE_NAME}

echo "Can interface '${CAN_INTERFACE_NAME}' is successfully set with bitrate '${BITRATE}'! 🎉"
