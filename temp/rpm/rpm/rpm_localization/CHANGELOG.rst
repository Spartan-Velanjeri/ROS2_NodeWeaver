^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rpm_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1
* Pull request #30: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/rpm from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '5ecc3dbaae5eb27a14bd17f34d0e55efc13738d3':
  Update Jenkinsfile
  Reformat some XML files
  Remove ament_python as build_type
  Use ament_cmake_python instead of ament_python
  Use feature branch for rpm_powertrain_driver
  Add ros2_canopen dependency
  Add missing rpm_powertrain_driver dependency
  Build geographic_transform from master
  Add geographic_transform
  Add ros_compat
  Try to build with upstream
  Add build status badge
  Add Jenkinsfile
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
* Reformat some XML files
* Remove ament_python as build_type
* Use ament_cmake_python instead of ament_python
* Pull request #26: topics names corrected
  Merge in BAUTIRO/rpm from feature/BAUTIRO-296 to develop
  * commit 'dd99f405fd47b805b3d2ccd8092b650a0bf6c085':
  topics names corrected
* commited by erz2lr@fus1: rpm_localization_launch.py
* Pull request #28: Feature/BAUTIRO-282 implement fine localization in behavior tree
  Merge in BAUTIRO/rpm from feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree to feature/BAUTIRO-378-test-in-schillerhoehe
  * commit '382284e2a77043af3bf5686495c41908f634613b':
  commited by ngq3fe: change config file of localization
  leica_frame_id
  move code in lu_rough_localization
* commited by ngq3fe: change config file of localization
* Merge branch 'feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
* leica_frame_id
* move code in lu_rough_localization
* topics names corrected
* Merge branch 'BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive
* Pull request #25: BAUTIRO-305 integration of diffdrive controller in rpm structure and test diffdrive
  Merge in BAUTIRO/rpm from BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive to develop
  * commit 'ea42d321c3c78a5bb266c70557cffcc1b50f1433':
  removed leica point from pgm file
  Yeahgit statusgit statusgit statusgit statusgit statusgit status! first time navigation in close loop on FUS1
  add slam map for le131
  add scripts
  add bringup launch for rpm
  create some launch files
  RPM_CAN_Mode_deactivation online editiert mit Bitbucket Bugfix: Output== CompareString
  Added new CAN and inverter start-up scripts
  merge rpm_description from Dennis Stogl into rpm.
  add config for rpm_actuators. Take over from Dennis Stogl
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive
* Pull request #23: BAUTIRO-15 create rpm launch files
  Merge in BAUTIRO/rpm from BAUTIRO-15-create-rpm-launch-files to develop
  * commit '7ec60fdfa44b8285c625396c0412219849535dba':
  process_ros_parameters as seperate function
  Add mode parameter
  Undo changes to manipulate_yaml.py
  tested with gazebo simulation (chk2le & erz2lr)
  tested on rpm-cu FUS1
  add LE131 map
  Use argument to switch bettween SIM and FUS1
  transfer  rpm_localization in #278 to #15
* Pull request #24: Feature/BAUTIRO-296
  Merge in BAUTIRO/rpm from feature/BAUTIRO-296 to develop
  * commit 'aee8d5431cdd92ed69a33f6e9e462228af4e957e':
  changed lidar names in manipulate.yaml
  Revert "changed topic names in manipulate.yaml"
  changed topic names in manipulate.yaml
* changed lidar names in manipulate.yaml
* Revert "changed topic names in manipulate.yaml"
  This reverts commit b4abe79cc702ef195f96cb9f531ca0a62abb9b05.
* changed topic names in manipulate.yaml
* Pull request #22: Feature/BAUTIRO-296
  Merge in BAUTIRO/rpm from feature/BAUTIRO-296 to develop
  * commit '5062350e2ed2b780b7d07e94cf4b92a06fdaf3a6':
  added imu topic param
  lidar_link_names reverted back
  changes made w.r.t lidar topic names in slam and urdf
* process_ros_parameters as seperate function
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* Add mode parameter
* Undo changes to manipulate_yaml.py
* tested with gazebo simulation (chk2le & erz2lr)
* tested on rpm-cu FUS1
* Yeahgit statusgit statusgit statusgit statusgit statusgit status! first time navigation in close loop on FUS1
* add slam map for le131
* add LE131 map
* changes made w.r.t lidar topic names in slam and urdf
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
* Use argument to switch bettween SIM and FUS1
* Merge branch 'BAUTIRO-15-create-rpm-launch-files' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* transfer  rpm_localization in #278 to #15
* Merge branch 'feature/BAUTIRO-278' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-278
* commited by erz2lr: changes during running up SLAM on FUS1 and final robot description
* Merge branch 'feature/BAUTIRO-278' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-278
* Pull request #18: Hotfix/BAUTIRO-245 resolve localization and navigation issues for develop branch
  Merge in BAUTIRO/rpm from hotfix/BAUTIRO-245-resolve-localization-and-navigation-issues-for-develop-branch to feature/BAUTIRO-278
  * commit 'b3896a08e760d6482d61bffb885de91f655bbc7b':
  changes in agruments
  changed upper cover back to normal collision
  changed footprint sized
  fixed rpm_localization
  collision for upper cover changed to box
  Added new maps and solved build for rpm_localization
* changed upper cover back to normal collision
* fixed rpm_localization
* Pull request #16: fixed typo
  Merge in BAUTIRO/rpm from feature/BAUTIRO-259 to develop
  * commit 'd1435b7a9290ae7e16aaacbf692dee669982ea33':
  fixed typo
* fixed typo
* Added new maps and solved build for rpm_localization
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
* Merge branch 'costmap_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* Pull request #12: Feature/BAUTIRO-147 setup and open loop test of navigation with fus1
  Merge in BAUTIRO/rpm from feature/BAUTIRO-147-setup-and-open-loop-test-of-navigation-with-fus1 to BAUTIRO-15-create-rpm-launch-files
  * commit '0cc7430b1a79e796d72c5f923e6670b9df445655':
  commited by ngq3fe: navigation works for 2 lidar on FUS1. Clibration of lidar front corrected.
  commited by ngq3fe: first nav2 parameters working with rear lidar.
  commited by ngq3fe: first setup for navigation on fus1
* Merge branch 'BAUTIRO-15-create-rpm-launch-files' of
  ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into
  feature/BAUTIRO-147-setup-and-open-loop-test-of-navigation-with-fus1
* Merge branch 'BAUTIRO-15-create-rpm-launch-files' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* name changes
* fus1_slam_lidar.yaml: set ouster_F/base_link_to_sensor and ouster_R/base_link_to_sensor to zeros
* fus1_slam_general.yaml: set tf/base_link_to_imu to zeros
* Working version
* Merge branch 'costmap_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into feature/enable_odom
* switch from c++ node to python node
* Pull request #11: Feature/BAUTIRO-128 upgrade maidemo gazebo model with the fus1 model
  Merge in BAUTIRO/rpm from feature/BAUTIRO-128-upgrade-maidemo-gazebo-model-with-the-fus1-model to feature/enable_odom
  * commit 'a20a6f3c271e5519650fe09ce64ab47eeaaede5e':
  enable odom by default to work with nav2
  add cr_slam to enable node
* First version "manipulate_yaml"
* commited by ngq3fe: navigation works for 2 lidar on FUS1. Clibration of lidar front corrected.
* changed lidar calibration in slam and tmux file
* commited by ngq3fe: first setup for navigation on fus1
* Launch localization
* add cr_slam to enable node
* Added a rosbag for cr_slam map
* rpm_localization laucnch file edited
* publish map to odom from slam
* Pull request #9: created a package called rpm_localization
  Merge in BAUTIRO/rpm from rpm_localization to feature/staging_develop
  * commit '98c7dffbb96751b46bdf5f70225d0302f1f2d694':
  created a package called rpm_localization
* created a package called rpm_localization
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Chandrahas Kasoju (PT/PJ-TOP100), Chandrahas_Kasoju, Erz Michael (CR/AAS5), Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro from rpm
