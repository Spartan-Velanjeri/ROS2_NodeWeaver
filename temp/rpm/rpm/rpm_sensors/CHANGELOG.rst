^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rpm_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Fix conflict
* Pull request #29: Feature/BAUTIRO-378 test in schillerhoehe
  Merge in BAUTIRO/rpm from feature/BAUTIRO-378-test-in-schillerhoehe to develop
  * commit '81f73796549acd0b0e68e5e178f3c7a4c9c35860':
  changes in urdf considering rpm1 vs rpm2
  change 378 specifics to develop
  fix problem with lidar plugin. Changes nav parameters
  commited by ngq3fe: clean nav2_params
  modify navigation rviz config
  commited by ngq3fe: clean navigation parameters
  changes in description to get simulation working with 378
  add navigation map for lab 131. ACHTUNG: in rpm_chassis_macro recht link ist in FUS2 anders wie FUS1
  commit by ngq3fe: change min_obstacle_height because map frame is on the sealing in schillerhoeher baustelle
  add maps for schillerhoeher
  add yaml file for le131 map
  add navigation map for shillerhoehe construction site
  commited by ngq3fe: add maps for navigation, add RPM2 description, change launch file to put controller_manager in namespace
  add navigation map for le131 and shillerhoeher construction site (generated from OBJ file)
  add calibration file for RPM2
  commited by erz2lr@fus1: rpm_localization_launch.py
  commited by ngq3fe: change config file of localization
  leica_frame_id
  add validation points into URDF
  move code in lu_rough_localization
* Pull request #28: Feature/BAUTIRO-282 implement fine localization in behavior tree
  Merge in BAUTIRO/rpm from feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree to feature/BAUTIRO-378-test-in-schillerhoehe
  * commit '382284e2a77043af3bf5686495c41908f634613b':
  commited by ngq3fe: change config file of localization
  leica_frame_id
  move code in lu_rough_localization
* Merge branch 'feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
* move code in lu_rough_localization
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* Pull request #20: Feature/BAUTIRO-278
  Merge in BAUTIRO/rpm from feature/BAUTIRO-278 to develop
  * commit '146645580c4183aea376f74b62802d57e45966f6':
  add velocity calculator to rpm_demo
  remove unnecessary collision bodies
  commited by erz2lr: changes during running up SLAM on FUS1 and final robot description
  delete joint from base_link to lift_base_mount in rpm_chassis
  changes in agruments
  changed upper cover back to normal collision
  changed footprint sized
  fixed rpm_localization
  collision for upper cover changed to box
  changes to fit robot model to FUS1
  commited by ngq3fe: changes for 278
  commited by ngq3fe: changes for 278
  Added new maps and solved build for rpm_localization
* Merge branch 'feature/BAUTIRO-278' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-278
* commited by erz2lr: changes during running up SLAM on FUS1 and final robot description
* Pull request #14: Costmap develop
  Merge in BAUTIRO/rpm from costmap_develop to develop
  * commit 'c6b3c968f54b2af3144ca0c8f3e15b9be2522826': (45 commits)
  name changes
  merge successful
  fus1_slam_lidar.yaml: set ouster_F/base_link_to_sensor and ouster_R/base_link_to_sensor to zeros
  fus1_slam_general.yaml: set tf/base_link_to_imu to zeros
  Working version
  switch from c++ node to python node
  First version "manipulate_yaml"
  commited by ngq3fe: navigation works for 2 lidar on FUS1. Clibration of lidar front corrected.
  changed lidar calibration in slam and tmux file
  commited by ngq3fe: first nav2 parameters working with rear lidar.
  Working nav2_params in simulation updated
  commited by ngq3fe: first setup for navigation on fus1
  commit by ngq3fe: change namespace, frames for ouster and imu. Tested in FUS1.
  Launch localization
  adjust Ouster IPs to FUS1
  Launch files for rpm_sensors; initial commit
  enable odom by default to work with nav2
  add cr_slam to enable node
  change ssh to https in dependencies.repos
  merged 2 lidars for costmap
  ...
* Pull request #13: BAUTIRO-15 create rpm launch files
  Merge in BAUTIRO/rpm from BAUTIRO-15-create-rpm-launch-files to costmap_develop
  * commit 'b0fadc0f4f3d6cb2d59f19727d4bc96280b0819f':
  name changes
  fus1_slam_lidar.yaml: set ouster_F/base_link_to_sensor and ouster_R/base_link_to_sensor to zeros
  fus1_slam_general.yaml: set tf/base_link_to_imu to zeros
  Working version
  switch from c++ node to python node
  First version "manipulate_yaml"
  commited by ngq3fe: navigation works for 2 lidar on FUS1. Clibration of lidar front corrected.
  commited by ngq3fe: first nav2 parameters working with rear lidar.
  commited by ngq3fe: first setup for navigation on fus1
  commit by ngq3fe: change namespace, frames for ouster and imu. Tested in FUS1.
  Launch localization
  adjust Ouster IPs to FUS1
  Launch files for rpm_sensors; initial commit
* Merge branch 'BAUTIRO-15-create-rpm-launch-files' of
  ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into
  feature/BAUTIRO-147-setup-and-open-loop-test-of-navigation-with-fus1
* name changes
* commit by ngq3fe: change namespace, frames for ouster and imu. Tested in FUS1.
* adjust Ouster IPs to FUS1
* Launch files for rpm_sensors; initial commit
* Contributors: Andreas Mogck, Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro from rpm
