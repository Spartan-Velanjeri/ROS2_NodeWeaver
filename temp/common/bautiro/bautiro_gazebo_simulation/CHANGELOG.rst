^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bautiro_gazebo_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #35: Feature/BAUTIRO-428 preparation autonomous driving for november demo with fus2
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus2 to develop
  * commit 'fd583bfb7fd6c5f32eeb27917ff7af8235c80796':
  update docker_sim_bringup_tmux
  commited by ngq3fe: change default name of FUS2
  update urdf with base_link in rotation axis
* update docker_sim_bringup_tmux
* Add FIXME comments
* Pull request #33: Feature/BAUTIRO-378 test in schillerhoehe
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-378-test-in-schillerhoehe to develop
  * commit 'b5285389941409751e86ab08174340e0622a561f':
  change fus2 urdf
  add comments
  add rviz node
  delete not used file
  rename 378 scripts to develop
  add tmux for simulation, fix simulation_rviz_config, and fix spawn position for LE131
  changes to get simulation running with 378
  mia4si: integrated into one shell script to launch
  commit by ngq3fe: add command for navigation and active CAN
  add navigation launch to tmux script
  commited by ngq3fe: add tmux file for loc only
  commited by ngq3fe: add FUS2 description
  commited by ngq3fe: change launch
  commited by erz2lr: bautiro_bringup/scripts/test378_rpm_bringup_tmux.sh
  commited by ngq3fe: change on sh construction site
  add tmux script for testing 378
  test282_fpm_bringup_tmux_NEW.sh test282_rpm_bringup_tmux_NEW.sh
  add workspace_center_point into URDF
  commited by erz2lr: first test behavior tree with new urdf
  commited by ngq3fe: changes for first test lu_bt with new FUS1 URDF
* add tmux for simulation, fix simulation_rviz_config, and fix spawn position for LE131
* changes to get simulation running with 378
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
* Pull request #31: edited required static transforms
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-296 to develop
  * commit '9e60eaabd801758e0b3c289b12a9895e0237eee9':
  edited required static transforms
* edited required static transforms
* Pull request #30: changed lidar topic names
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-296 to develop
  * commit '6495fb169decda63160aee9861b66b355917b7ca':
  added "scan" namespace to laserscan topic
  static transforms of lidar reverted to old versions
  changed lidar topic names
* added "scan" namespace to laserscan topic
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-296
* static transforms of lidar reverted to old versions
* changed lidar topic names
* Pull request #29: Feature/BAUTIRO-278
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-278 to develop
  * commit '253781ad786a819beda6142217007493b0acfa4f':
  removed joint state publisher
  added robot description publisher to bring_up
  add readme for robot description
  aligned reading robot description
  add handling_unit_base for in launch file
  changes for ur_description
  small changes in launch files
  changes to fit robot model to FUS1
  commied by ngq3fe: changes for 278
  commited by ngq3fe: changes for 278
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/BAUTIRO-278
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into develop
* added robot description publisher to bring_up
* Merge branch 'feature/BAUTIRO-278' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-278
* aligned reading robot description
* add handling_unit_base for in launch file
* changes for ur_description
* small changes in launch files
* Pull request #28: Added new world with obstacles
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-284-add-new-worlds-for-gazebo to develop
  * commit 'd0e202fdb94b9ae9a96baca2ed797b8d1ef43fb4':
  Added new world with obstacles
* Added new world with obstacles
* changes to fit robot model to FUS1
* commied by ngq3fe: changes for 278
* Pull request #24: Feature/staging develop
  Merge in BAUTIRO/bautiro from feature/staging_develop to develop
  * commit '83a1c0b0325bb1b0f423d88832f28b3871d3811e': (80 commits)
  updated documentation
  removed outdated FUS0 description
  removed old files
  updated readme for systemdays demo may 2023
  add tmux bringup script
  update launch files with offset frames data service node
  cleaned gazebo launch file
  added ur_base_new static transform
  demo_systemdays_may2023.md online editiert mit Bitbucket
  commited by ngq3fe: correct calibration for lidar front in FUS1
  front lidar calibrated as in FUS1
  commited by ngq3fe: first configuration nav2 on fus1
  demo_systemdays_may2023.md online editiert mit Bitbucket
  working bt and cr_slam
  add cr_slam content to bt conent
  changed z pos for safe spawning of robot
  add data service node to bringup launch file
  test successful, ready to merge
  Change version of ccu,fpm and rpm in dependencies.repos
  change ssh to https
  ...
* Pull request #27: Feature/working bt
  Merge in BAUTIRO/bautiro from feature/working_bt to feature/staging_develop
  * commit '946cfc8651ad93a25de2c19679afaf73288872f0':
  updated readme for systemdays demo may 2023
  demo_systemdays_may2023.md online editiert mit Bitbucket
  commited by ngq3fe: correct calibration for lidar front in FUS1
  commited by ngq3fe: first configuration nav2 on fus1
  demo_systemdays_may2023.md online editiert mit Bitbucket
  working bt and cr_slam
  add cr_slam content to bt conent
  add data service node to bringup launch file
  update bt lifecycle launch file
  update pkg names in launch files
  update to new lu fpm bt
  update readme
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/working_bt
* Pull request #23: Feature/BAUTIRO-147 setup and open loop test of navigation with fus1
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-147-setup-and-open-loop-test-of-navigation-with-fus1 to feature/working_bt
  * commit '2fb737567217e8a1d633494d7ec4828b0404f2d8':
  commited by ngq3fe: correct calibration for lidar front in FUS1
  commited by ngq3fe: first configuration nav2 on fus1
  changed z pos for safe spawning of robot
  test successful, ready to merge
  Change version of ccu,fpm and rpm in dependencies.repos
  change ssh to https
  demo_systemdays_may2023.md edited online with Bitbucket
  new world with obstacles added
  uncommented laser_scan topics
  added static transform for imu and callibrated rpm
  Changed lidar topic names and static transforms
  correct syntax error in dependencies.repo
  modification for sensorrack macro
  Got the lidars to running
  callibrated sensors directly from base link
* Merge branch 'feature/working_bt' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/BAUTIRO-147-setup-and-open-loop-test-of-navigation-with-fus1
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* update launch files with offset frames data service node
* cleaned gazebo launch file
* added ur_base_new static transform
* Pull request #20: Feature/BAUTIRO-128 upgrade maidemo gazebo model with the fus1 model
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-128-upgrade-maidemo-gazebo-model-with-the-fus1-model to feature/working_bt
  * commit 'b59679f291bfaa1fa095ca9f6f9f0a4ff5987c53':
  demo_systemdays_may2023.md online editiert mit Bitbucket
  working bt and cr_slam
  add cr_slam content to bt conent
* commited by ngq3fe: first configuration nav2 on fus1
* working bt and cr_slam
* add cr_slam content to bt conent
* changed z pos for safe spawning of robot
* update pkg names in launch files
* new world with obstacles added
* uncommented laser_scan topics
* added static transform for imu and callibrated rpm
* Changed lidar topic names and static transforms
* Pull request #18: callibrated sensors directly from base link
  Merge in BAUTIRO/bautiro from rpm_calibration to feature/staging_develop
  * commit '848743538755223bd35cc7ed5899fc53af99f13d':
  modification for sensorrack macro
  Got the lidars to running
  callibrated sensors directly from base link
* Merge branch 'rpm_calibration' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into rpm_calibration
* Got the lidars to running
* callibrated sensors directly from base link
* removed gazebo gui
* empty lab and proper naming
* update node names in launch files
* modified urdf and launch file to pub base_link
* review with michael and huy
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* gazebo is working with new urdf model
* reuse accidentally deleted nodes
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* Working Navigation
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* robot description variants and calibration files
* update launch file
* uncommented laser scan topics from lidar
* updated launch file to for fake nav loc
* Modify launch file variable names
* correct deleted file
* Pull request #14: Bulk movement from testing branch to develop branch
  Merge in BAUTIRO/bautiro from prr1le/testing to develop
  * commit 'a1f9ab06f4c10b0dd8914a55ec0de0cd1a029c98':
  uncommented realsense urdf
  removed behavior tree from testing branch
  added behavior tree for publishing commands and checking with kpi
  update readme
  update for readme
  added README for bautiro_gazebo_simulation
  added rpm and fpm in the drawing
  minor block alignemts
  included realsense, added headless launch argument for gazebo, updated drawing for gazebo model with plugins and topics
  added inclinometer
  initial diagramm sensor plugins in gazebo
  added leica, made name changes as suggested
  spawn position and transforms for markers
  added plugin in the sdf and urdf, added two new topics in ros ign bridge node in launch file
  added leica plugin in urdf
  revert back test changes
  test
* update readme
* update for readme
* added README for bautiro_gazebo_simulation
* added rpm and fpm in the drawing
* minor block alignemts
* included realsense, added headless launch argument for gazebo, updated drawing for gazebo model with plugins and topics
* added headless mode launch feature for gazebo
* added headless launch for gazebo
* Merge branch 'prr1le/testing' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into prr1le/testing
* initial diagramm sensor plugins in gazebo
* added leica, made name changes as suggested
* spawn position and transforms for markers
* added plugin in the sdf and urdf, added two new topics in ros ign bridge node in launch file
* added leica plugin in urdf
* added blank readme files for ros packages
* Pull request #9: added rviz launch argument for bautiro bringup
  Merge in BAUTIRO/bautiro from prr1le/testing to release/systemdays2022
  * commit 'bcfb749ca58baa159b42bc584b3355ccaad567d9':
  added if condition for rviz node in bautiro gazebo launch file
  added node for leica pose publish
  added rviz launch argument for bautiro bringup
* Pull request #8: added rviz launch argument for bautiro bringup
  Merge in BAUTIRO/bautiro from prr1le/testing to develop
  * commit 'bcfb749ca58baa159b42bc584b3355ccaad567d9':
  added if condition for rviz node in bautiro gazebo launch file
  added node for leica pose publish
  added rviz launch argument for bautiro bringup
* update readme for demo systemdays2022
* added if condition for rviz node in bautiro gazebo launch file
* added node for leica pose publish
* Pull request #7: Merge request for sensors, odom and world addition
  Merge in BAUTIRO/bautiro from prr1le/testing to develop
  * commit '4ff16c2108a192533bb54c6ffc2d288df0f8f885':
  updated rviz simulation settings
  README.md edited online with Bitbucket
  README.md edited online with Bitbucket
  update
  added relative path for world file
  update
  update for world and joint state broadcaster
  added sensors and world
* updated rviz simulation settings
* update
* added relative path for world file
* update for world and joint state broadcaster
* added sensors and world
* Pull request #6: Feature/bring up moveit
  Merge in BAUTIRO/bautiro from feature/bring_up_moveit to develop
  * commit '532159263cef81ecd2147eb055a60ace0957935c':
  adaption for MoveIT
  bautiro spawns again in gazebo and all controllers are running!
  added bringup package
* bautiro spawns again in gazebo and all controllers are running!
* added bringup package
* Pull request #4: Feature/merged robot model
  Merge in BAUTIRO/bautiro from feature/merged_robot_model to develop
  * commit 'eb3ee981e741df44628e00e580d5ae4d31cf0d18':
  moved file path ro launch file and modified readme
  removed todo's
  merged rpm, fpm, ur
  update
* moved file path ro launch file and modified readme
* merged rpm, fpm, ur
* update
* Pull request #3: Feature/merged robot model
  Merge in BAUTIRO/bautiro from feature/merged_robot_model to develop
  * commit '02ea9ca84890ad0460fd55053cc83a8c46805f02':
  revert back removed gazebo
  added controller
  modified simulation launch file to control gazebo plugins
  included plugin to launch directly
  WIP initial files for merging rpm and fpm, not working
* Merge branch 'feature/merged_robot_model' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/merged_robot_model
* modified simulation launch file to control gazebo plugins
* WIP initial files for merging rpm and fpm, not working
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Andreas.Mogck, Chandrahas Kasoju (PT/PJ-TOP100), Chandrahas_Kasoju, Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Premkumar Raamkishore (PT/PJ-TOP100), bautiro from rpm, mia4si, prr1le
