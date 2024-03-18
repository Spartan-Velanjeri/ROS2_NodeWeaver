^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bautiro_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #35: Feature/BAUTIRO-428 preparation autonomous driving for november demo with fus2
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus2 to develop
  * commit 'fd583bfb7fd6c5f32eeb27917ff7af8235c80796':
  update docker_sim_bringup_tmux
  commited by ngq3fe: change default name of FUS2
  update urdf with base_link in rotation axis
* update docker_sim_bringup_tmux
* add launch rpm behaviour tree in sim_brungup_tmux
* Add FIXME comments
* Pull request #34: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/bautiro from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '298e93d3f812532af248f14dd0bfe6ee1cdebd43': (30 commits)
  Remove backward_ros from dependencies.repos
  Use local profile to source all env vars
  Remove repos already installed in image
  Set local external ws as additional env
  Source local 3rd party deps workspace
  Run entrypoint with bash
  Run entrypoint with sh
  Run as script
  Fix typo
  Run entrypoint as pre build dep
  Disable entrypoint override
  Fix typo
  Run local entrypoint script
  Enable entrypoint
  Fix typo
  Force run entrypoint
  Enable tests
  Add backward_ros to dependency list
  Add rpm_powertrain_driver to dependencies list
  Add ros2_canopen to dependencies list
  ...
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
* delete not used file
* rename 378 scripts to develop
* add tmux for simulation, fix simulation_rviz_config, and fix spawn position for LE131
* Add missing runtime dependencies to bautiro_bringup
* Remove whitespaces
* Remove typo
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-378-test-in-schillerhoehe
* mia4si: integrated into one shell script to launch
* commit by ngq3fe: add command for navigation and active CAN
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-378-test-in-schillerhoehe
* add navigation launch to tmux script
* commited by ngq3fe: add tmux file for loc only
* commited by ngq3fe: change launch
* Merge branch 'feature/BAUTIRO-378-test-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-378-test-in-schillerhoehe
* commited by erz2lr: bautiro_bringup/scripts/test378_rpm_bringup_tmux.sh
* commited by ngq3fe: change on sh construction site
* add tmux script for testing 378
* Pull request #32: Feature/BAUTIRO-282 implement fine localization in behavior tree
  Merge in BAUTIRO/bautiro from feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree to feature/BAUTIRO-378-test-in-schillerhoehe
  * commit 'a7662df0555b4b3497ac736ab16ed72a3fb651e6':
  test282_fpm_bringup_tmux_NEW.sh test282_rpm_bringup_tmux_NEW.sh
  commited by erz2lr: first test behavior tree with new urdf
  commited by ngq3fe: changes for first test lu_bt with new FUS1 URDF
* test282_fpm_bringup_tmux_NEW.sh test282_rpm_bringup_tmux_NEW.sh
* Merge branch 'feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro into feature/BAUTIRO-282-implement-fine-localization-in-behavior-tree
* commited by erz2lr: first test behavior tree with new urdf
* commited by ngq3fe: changes for first test lu_bt with new FUS1 URDF
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
* removed joint state publisher
* added robot description publisher to bring_up
* changes to fit robot model to FUS1
* add tmux script for launcing in docker
* changed data service executable name
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
* removed old files
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
* add tmux bringup script
* update launch files with offset frames data service node
* commited by ngq3fe: first configuration nav2 on fus1
* add data service node to bringup launch file
* update bt lifecycle launch file
* update pkg names in launch files
* update to new lu fpm bt
* update readme
* update bt tolaunch with lifecycle manager
* update sim nodes launch to move ur base
* update launch file name in readme
* update readme
* update demo launch files
* empty lab and proper naming
* use poses from ccu data service
* add tree node to launch file
* update node names in launch files
* added launch file for sim nodes
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* modified urdf and launch file to pub base_link
* update fake nav launch file
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro into feature/staging_develop
* updated bringup for fake nav
* added launch arguments robot name
* add launch file for fake nav loc system
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
* added rviz launch argument for bautiro bringup
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
* update
* added relative path for world file
* Pull request #6: Feature/bring up moveit
  Merge in BAUTIRO/bautiro from feature/bring_up_moveit to develop
  * commit '532159263cef81ecd2147eb055a60ace0957935c':
  adaption for MoveIT
  bautiro spawns again in gazebo and all controllers are running!
  added bringup package
* adaption for MoveIT
* bautiro spawns again in gazebo and all controllers are running!
* added bringup package
* Contributors: Ahmad Abouelainein, Andreas Mogck, Chandrahas_Kasoju, Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Premkumar Raamkishore (PT/PJ-TOP100), bautiro, bautiro from rpm, mia4si, prr1le
