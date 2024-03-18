^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rpm_nav2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #31: Feature/BAUTIRO-428 preparation autonomous driving for november demo with fus1
  Merge in BAUTIRO/rpm from feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1 to develop
  * commit '309e23e5844bedf2d177f6779e3b42769f35df74':
  add tmux script for navigation
  commited by ngq3fe: changes for test with FUS1
  add scripts and some parameters change. Tested with RPM2 on schillerhöher construction site
  commited by ngq3fe: move base_link to rotation axis. ca. 16cm from old base_link, which is now rpm_base_link
  commited by ngq3fe: changes in tmux for demo. add scripts for rpm test with CAN
  commited by ngq3fe: change tmux script for NovDemo
  commited by ngq3fe: final preparation of NovDemo
  add calibrated value of working space center point
  get back to udrf without rotation axis
  update rpm urdf with base_link in rotation axis
  commited by ngq3fe: change params
  update tmux script
  erz2lr: today test in Le on FUS2
  commited by ngq3fe: change urdf to RPM2
  add working space center point
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1
* add tmux script for navigation
* add scripts and some parameters change. Tested with RPM2 on schillerhöher construction site
* commited by ngq3fe: move base_link to rotation axis. ca. 16cm from old base_link, which is now rpm_base_link
* commited by ngq3fe: changes in tmux for demo. add scripts for rpm test with CAN
* commited by ngq3fe: final preparation of NovDemo
* get back to udrf without rotation axis
* commited by ngq3fe: change params
* enlarge footprint for more stable object detection
* erz2lr: today test in Le on FUS2
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
* changes in urdf considering rpm1 vs rpm2
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* fix problem with lidar plugin. Changes nav parameters
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* commited by ngq3fe: clean nav2_params
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* modify navigation rviz config
* commited by ngq3fe: clean navigation parameters
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* changes in description to get simulation working with 378
* add navigation map for lab 131. ACHTUNG: in rpm_chassis_macro recht link ist in FUS2 anders wie FUS1
* commit by ngq3fe: change min_obstacle_height because map frame is on the sealing in schillerhoeher baustelle
* add maps for schillerhoeher
* add yaml file for le131 map
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* add navigation map for shillerhoehe construction site
* commited by ngq3fe: add maps for navigation, add RPM2 description, change launch file to put controller_manager in namespace
* add navigation map for le131 and shillerhoeher construction site (generated from OBJ file)
* add calibration file for RPM2
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
* Pull request #27: add fpm to xarcro file
  Merge in BAUTIRO/rpm from BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive to develop
  * commit 'd1b1a6acbe892e49c12bcf08937dc4520c92c183':
  add fpm to xarcro file
* Merge branch 'BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-305-integration-of-diffdrive-controller-in-rpm-structure-and-test-diffdrive
* add fpm to xarcro file
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
* Pull request #22: Feature/BAUTIRO-296
  Merge in BAUTIRO/rpm from feature/BAUTIRO-296 to develop
  * commit '5062350e2ed2b780b7d07e94cf4b92a06fdaf3a6':
  added imu topic param
  lidar_link_names reverted back
  changes made w.r.t lidar topic names in slam and urdf
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* removed leica point from pgm file
* Yeahgit statusgit statusgit statusgit statusgit statusgit status! first time navigation in close loop on FUS1
* add scripts
* lidar_link_names reverted back
* add bringup launch for rpm
* create some launch files
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
* Merge branch 'BAUTIRO-15-create-rpm-launch-files' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
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
* changes in agruments
* changed footprint sized
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
* merge successful
* Pull request #7: Feature/enable odom
  Merge in BAUTIRO/rpm from feature/enable_odom to costmap_develop
  * commit 'ba7e6dbfe8e910a6b34762b98d01c2fd45dddafc':
  enable odom by default to work with nav2
  add cr_slam to enable node
  add enable_odom_tf flag in rpm_controller_launch file
* Merge branch 'costmap_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into feature/enable_odom
* Pull request #11: Feature/BAUTIRO-128 upgrade maidemo gazebo model with the fus1 model
  Merge in BAUTIRO/rpm from feature/BAUTIRO-128-upgrade-maidemo-gazebo-model-with-the-fus1-model to feature/enable_odom
  * commit 'a20a6f3c271e5519650fe09ce64ab47eeaaede5e':
  enable odom by default to work with nav2
  add cr_slam to enable node
* commited by ngq3fe: navigation works for 2 lidar on FUS1. Clibration of lidar front corrected.
* commited by ngq3fe: first nav2 parameters working with rear lidar.
* Working nav2_params in simulation updated
* commited by ngq3fe: first setup for navigation on fus1
* add cr_slam to enable node
* merged 2 lidars for costmap
* Changed obstacle layer configuration
* Added a launch file for the map_server
* integrated cr_slam with Nav2
* removed voxel layer filter from local costmap
* publish map to odom from slam
* delete unused launch files
* Add launch, map and vonfiguration file to rpm_nav2
* - add dependencies.repos
  - add rpm_nav2 package
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Chandrahas Kasoju (PT/PJ-TOP100), Chandrahas_Kasoju, Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro from rpm, bautiro on rpm2
