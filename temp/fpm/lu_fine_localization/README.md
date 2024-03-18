# Fein localization

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/lu_fine_localization/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/lu_fine_localization/actions/workflows/build.yml)

---
## Required Repos/Packages

```bash
pip install scipy==1.5.2 
```
For ```lu_fine_localization/triangulation_node.py```, line 97, ```R_BIM_leica = Rotation.from_dcm(T_BIM_leica[:3,:3])```


--- 


## ROS Packages

Packages for Robot Operating System (ROS2)

## Configuration

### Core Marker List
```lu_fine_localization/tf2_reflex_marker_broadcaster.py```

```Python
fname_asc = '/home/bautiro/driver_ws/src/lu_fine_localization/config/Bosch_Le_131_Festpunkte_SLAM-MappingRun.asc'
```

## Usage

```Bash
ros2 launch lu_fine_localization bringup_fine_sensors.launch.py
ros2 run lu_fine_localization fein_loc_coordination
...
```

## Usage fine_localization service

```bash

ros2 service call /lu_fine_localization lu_fine_localization/srv/LuFineLocalization "
{
    markers: [
        {
            marker_id: 'm1',
            pose: {pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
        }
    ]
}
"



```
