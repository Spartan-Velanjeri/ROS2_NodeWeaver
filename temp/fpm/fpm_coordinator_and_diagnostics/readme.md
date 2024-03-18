# FPM coordinator and diagnostics repository

[![Build and test packages](https://github.boschdevcloud.com/BAUTIRO/fpm_coordinator_and_diagnostics/actions/workflows/build.yml/badge.svg)](https://github.boschdevcloud.com/BAUTIRO/fpm_coordinator_and_diagnostics/actions/workflows/build.yml)

# FPM Coordinator and Diagnostics

The repository contains the following modules:

- FPM [coordinator](coordinator/readme.md) module that abstracts operation modes of the FPM (fine positioning module). Implemented as ROS 2 package.

## Repository Structure

Repository implements the following modules:

| Folder/File         |                              Description                              |
| ------------------- | :-------------------------------------------------------------------: |
| coordinator         | FPM coordinator module that implements state/mode machine of the FPM. |
| coordinator/doc     |             Documentation of the FPM coordinator module.              |
| doc                 |         Folder that contains documentation of the repository.         |
| readme.md           | Description of the repository _(not used for (sphinx) documentation)_ |
| index.rst           |     Entry-point for sphinx documentation _(include from parent!)_     |
| dependencies.repos  |       Source dependencies for managing workspace using vcstool        |
| jenkinsfile_bautiro |     Jenkins job for continuos integration don not change content      |

### Repository structure of the software modules

The folder/file structure of the software modules is as in the table below.

| File          | Location  | Description                                                           |
| ------------- | :-------: | --------------------------------------------------------------------- |
| readme.md     | top-level | Description of the repository _(not used for (sphinx) documentation)_ |
| index.rst     |   doc/    | Entry-point for sphinx documentation _(include from parent!)_         |
| COLCON_IGNORE | top-level | Empty file to skip colcon build process.                              |
