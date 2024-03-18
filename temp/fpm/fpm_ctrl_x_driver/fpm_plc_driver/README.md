[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat&labelColor=ef8336)](https://pycqa.github.io/isort/)

# fpm_plc_driver

## Overview

ROS Package (python) for controlling motion of the lift and controlling vacuum system. The ROS-Nodes communicate with the ctrlX PLC (Rexroth). The ROS-Nodes can be used only in combination with the PLC in which is loaded the corresponding PLC program.

## Documentation

[Overview_sw_package](doc/overview_sw_package.md)

## Installation

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

- ❗The Package depends on definitions from the bautiro_common [Repo](https://sourcecode.socialcoding.bosch.com/projects/BAUTIRO/repos/bautiro_common)
- Build ROS2 package.

```
colcon build --packages-select fpm_plc_driver
```

## How to run

```
ros2 run fmp_plc_driver lift_driver
ros2 run fmp_plc_driver vacuum_driver
```
