# README

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/lu_rough_localization/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/lu_rough_localization/actions/workflows/build.yml)

## Maps

see [README.md](maps/README.md)

## Test

### transform service

```bash
ros2 service call /lu_get_trafo bautiro_ros_interfaces/srv/GetTrafo "{from_frame: {data: 'map'}, to_frame: {data: 'map'} }"
```

### SLAM launch

```bash
. scripts/test_with_static_robot_description_for_docker.sh
```
