# Run

```bash
ros2 launch rpm_sensors rpm_sensors_fus1_static_urdf_launch.py
ros2 launch rpm_localization rpm_localization_static_urdf_launch.py
```

or

```bash
ros2 launch bautiro_description display_robot_description.launch.py 
ros2 launch rpm_sensors rpm_sensors_launch.py
source ~/bautiro_ws/install/setup.bash && ros2 run lu_fein_localization transform_services
ros2 launch rpm_localization rpm_localization_launch.py
```

```bash
ros2 launch bautiro_description display_robot_description.launch.py 
ros2 launch rpm_sensors rpm_sensors_launch.py
ros2 launch rpm_localization rpm_localization_launch.py mode:=real

# source /bautiro_environment/278_bautiro_core/common/install/setup.bash
# source /home/bautiro/testws/install/setup.bash
```
