^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rpm_actuators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* add scripts and some parameters change. Tested with RPM2 on schillerhöher construction site
* get back to udrf without rotation axis
* Merge branch 'feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1
* update rpm urdf with base_link in rotation axis
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
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-378-test-in-schillerhoehe
* commited by ngq3fe: add maps for navigation, add RPM2 description, change launch file to put controller_manager in namespace
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/rpm into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
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
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* create some launch files
* RPM_CAN_Mode_deactivation online editiert mit Bitbucket Bugfix: Output== CompareString
* Added new CAN and inverter start-up scripts
* add config for rpm_actuators. Take over from Dennis Stogl
* add package rpm_actuators
* Contributors: FAE2ABT, Felgentraeger Arne (BEG/CWW3-OR), Michael Erz (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro on rpm2
