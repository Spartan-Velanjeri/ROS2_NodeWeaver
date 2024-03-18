# Functional repository template

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/rpm_powertrain_driver/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/rpm_powertrain_driver/actions/workflows/build.yml)

**Repository Files**:

| File                | Location  | Description                                                           |
| ------------------- | :-------: | --------------------------------------------------------------------- |
| readme.md           | top-level | description of the repository *(not used for (sphinx) documentation)* |
| index.rst           |    doc    | entry-point for sphinx documentation  *(include from parent!)*        |
| dependencies.repos  | top-level | source dependencies for managing workspace using vcstool              |
| jenkinsfile_bautiro | top-level | Jenkins job for continuos integration don not change content          |

**SW packages structure**:

One or more SW packages in dedicated folders , ROS2 or other:

| File          | Location  | Description                                                           |
| ------------- | :-------: | --------------------------------------------------------------------- |
| readme.md     | top-level | description of the repository *(not used for (sphinx) documentation)* |
| index.rst     |   doc/    | entry-point for sphinx documentation *(include from parent!)*         |
| COLCON_IGNORE | top-level | Empty file to skip colcon build process,                              |

## Template repository readme.md

## Bautiro template function repository

Very brief description of this repository:
This is a template for a functional repository of the BAUTIRO project

---

## Documentation

[doc](doc)  

---

## Packages

### [non_ros_package](non_ros_package/readme.md) package 

very brief description of the package.

---

## ROS Packages

Packages for Robot Operating System (ROS2)

### [ros_package_1](ros_package_1/readme.md) package

very brief description of the package.

