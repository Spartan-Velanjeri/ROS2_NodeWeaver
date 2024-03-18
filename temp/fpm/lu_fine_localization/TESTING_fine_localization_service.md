# How to test/debug fine localization service

**Terminal**
```bash
ros2 launch bautiro_description display_robot_description.launch.py
```

**Terminal**
```bash
# Emulated position at (0,0,0)
ros2 service call /fine_localization lu_fine_localization/srv/LuFineLocalization "
{ markers: 
    [{
        marker_id: 'm1',
        pose: {pose: {position: {x: 1, y: 0.0, z: -0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: 0.0, y: 1.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: -1, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    }]
}
"
```

```bash
# Emulated position at (0,0,0)
ros2 service call /fine_localization lu_fine_localization/srv/LuFineLocalization "
{ markers: 
    [{
        marker_id: 'm1',
        pose: {pose: {position: {x: -19.612754, y: 18.380783, z: 2.241907}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: -29.590328, y: 10.290782, z: 1.059593}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    },{
        marker_id: 'm2',
        pose: {pose: {position: {x: -29.583681, y: 2.790075, z: 2.182058}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
    }]
}
"
```


<!-- 
1310008 -19.612754 18.380783 2.241907
1310009 -29.590328 10.290782 1.059593
1310010 -29.583681  2.790075 2.182058 
-->
