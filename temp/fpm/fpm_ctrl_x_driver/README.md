[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/fpm_ctrl_x_driver/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/fpm_ctrl_x_driver/actions/workflows/build.yml)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat&labelColor=ef8336)](https://pycqa.github.io/isort/)

# fpm_plc_driver
ROS Package (python) for controlling the devices that are connected to the PLC.
* lift motion
* vacuum system
* ptu

## Table of content
- [fpm\_plc\_driver](#fpm_plc_driver)
  - [Table of content](#table-of-content)
  - [Overview](#overview)
  - [Folder structure](#folder-structure)
  - [Installation (Ctrl-x) - abandoned](#installation-ctrl-x---abandoned)
  - [How to build](#how-to-build)
  - [How to run](#how-to-run)



## Overview

The ROS-Nodes communicate with the Beckhoff PLC and can be used only in combination with the PLC in which is loaded the corresponding PLC program.

## Folder structure
```
├── config
│   ├── test_lift_ready.yaml            # parameters for test
├── doc                                 # documentation folder
├── fpm_plc_driver
|   ├── ads_client_ptu.py               # ads client for ptu unit
|   ├── ads_client.py                   # ads client for lift
|   ├── ads_logger.py                   # logger for module
|   ├── dlClient.py                     # ctrl-x client (not used any longer, used with ctrl-x plc)
|   ├── drive_state.py                  # enumerations for drive state
|   ├── error_codes_lift_drive.py       # error codes of the lift drive
|   ├── lift_driver.py                  # ROS driver for lift
|   ├── ptu_driver.py                   # ROS driver for ptu
|   ├── state.py                        # state of the lift
|   ├── vacuum_driver.py                # driver for the vacuum
├── launch
|   ├── lift.launch.py                  # launch for lift node 
|   ├── main.launch.py                  # launch for all plc nodes
|   ├── ptu.launch.py                   # launch for ptu node
|   ├── vacuum.launch.py                # launch for vacuum                              
├── tests
|   ├── conftest.py                     # configuration for the test
|   ├── test_ads_lift_driver.py         # test on the real system (hw)
|   ├── test_ads_ptu_driver.py          # test on the real system (hw)
|   ├── test_copyright.py               # copyright test
|   ├── test_ctrl_x_drive_init.py       # ctrl-x plc driver test (not used)
|   ├── test_ctrlx_drive_motion.py      # ctrl-x plc driver test (not used)
|   ├── test_flake8.py                  # linter test
|   ├── test_lift_ros_driver.py         # lift ros driver test
└── .gitignore
```



## Installation (Ctrl-x) - abandoned

- Instal ctrlx data-layer python and deb packages
- Check [Link](https://pypi.org/project/ctrlx-datalayer/) for the latest information

```
    sudo apt-get update
    sudo apt-get
    DATALAYER_DEB_VERSION=1.10.7
    SDK_RELEASE_VERSION=1.20.0
    wget --quiet https://github.com/boschrexroth/ctrlx-automation-sdk/releases/download/${SDK_RELEASE_VERSION}/ctrlx-datalayer-${DATALAYER_DEB_VERSION}.deb
    sudo dpkg --install ctrlx-datalayer-${DATALAYER_DEB_VERSION}.deb
    pip3 install ctrlx-datalayer
```

## How to build
Build ROS2 package and its dependencies.

```
colcon build --packages-up-to fpm_plc_driver
```

## How to run

```
ros2 run fmp_plc_driver lift_driver
ros2 run fmp_plc_driver vacuum_driver
```