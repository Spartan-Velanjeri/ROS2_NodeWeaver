# FPM Repository

Integration/Meta package of the fine positioning module (FPM)

---
## Documentation


[FPM Overview](doc/fpm_overview.md)
[FPM Drill sequence and control](doc/fpm_drill_sequence_and_control.md)
[FPM Handling Unit](doc/fpm_handling_unit_control.md)

---

## Required Repos/Packages

**! Info** please use vcstool with:
- [dependencies.repos](dependencies.repos) 

to get all software packages of the FPM required to operate the module. Hence software such as measurement analysis programs for the test benches are excluded.   

---

## Packages

currently no common packages 

--- 


## ROS Packages

Packages for Robot Operating System (ROS2)

### [fpm_bringup](fpm_bringup/README.MD) package 

Contains launchfiles to start the fpm modules and configuration files.

### [fpm_control](fpm_control/README.MD) package

Contains all files for configure the controllers used within ROS (such as controller.yaml)

### [fpm_description](fpm_description/README.MD) package

Xacro files with urdf, ros2_control and gazebo tags also launch files to view the robot model and tf. 

### [fpm_demo](fpm_demo/README.MD) package

Demonstrate dedicated functions of the FPM, minimal working setup not aligned with the entire SW architecture!

### [fpm_moveit](fpm_moveit/README.MD) package

Demonstrate dedicated functions of the FPM, minimal working setup not aligned with the entire SW architecture!


### [fpm_gazebo_simulation](fpm_gazebo_simulation/README.MD) package

Launch files for world files for simulating the fpm moduel. Only  starting gazebo and spawning the fpm, no fpm function within this package!
