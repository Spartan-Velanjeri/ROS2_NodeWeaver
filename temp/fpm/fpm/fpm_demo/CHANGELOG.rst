^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm into feature/BAUTIRO-518-simulation-mode-for-ts16
* Pull request #26: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/fpm from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '59e93fb809f0b2f3d787ce90d2c34f837c6a7405':
  Use Groot installation from CI image
  Remove dependencies that are installed in the Docker image
  Source local profile
  Use develop branch
  Add missing dependencies
  Add ur_msgs to list of dependencies
  Use feature branch for fpm_motion_manager
  Add bautiro_common as a dependency
  Pull the galactic branch from moveit2
  Add Jenkinsfile
* Add missing dependencies
* Merge branch 'develop' into BAUTIRO-50-bringup-files
* Pull request #19: Feature/BAUTIRO-285 moveit configuration for ur using prefix
  Merge in BAUTIRO/fpm from feature/BAUTIRO-285-moveit-configuration-for-ur-using-prefix to develop
  * commit 'ecc5902816d89958308a9353513dbef8ceec1fe9':
  changes for hu_ur_6axis
  planning with RVIZ is working
  adapted velocity and tested joint poses on FUS1
  added inital poses
  change values of fpm_acr_hum, sothat y-axis of hu_base point down like in FUS1
  changes fpm_acr_hum calibration value, so that y-axis of hu_base point down like in FUS1
  modified rotation of hu mount
  added launch file for rviz
  undo Launchargument for load_yaml
  changes for changes in move_group name
  add visual and collision body for vacuum cleaner, leica and use hub
  modifications for prefix
  changes in stl file for lift segments
  small changes
  urdf changes
  changes to fit robot model to FUS1
  commited by ngq3fe: change for 278
* changes for hu_ur_6axis
* modifications for prefix
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm into feature/BAUTIRO-124-create-fpm-launch-files
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into feature/staging_develop
* Pull request #12: deleted redundant dependencies
  Merge in BAUTIRO/fpm from fix/dependencies to develop
  * commit 'aac3e59acea621dc41a92a803f755d99c7f4494d':
  blue plate aboce the lift mounted with ur is visible in rviz now
  deleted redundant dependencies
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into fix/dependencies
* fixed duplicates
* deleted redundant dependencies
* Pull request #11: Bulk movement from testing branch to develop branch
  Merge in BAUTIRO/fpm from prr1le/testing to develop
  * commit '170685de72e73523030fc2216a94b73cb34f0a38':
  changes according review
  changed forward to lift command controller
  added hollow cad models to implement collision in lift
  added joints and new cad model
  added meshes
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into prr1le/testing
* Pull request #9: Feature/mini coordinator demo systemdays
  Merge in BAUTIRO/fpm from feature/mini_coordinator_demo_systemdays to develop
  * commit 'b8fb25a7c411cb338f5a0fd6ac8b4051955da115':
  rempved mini_coordinator
  final changes before merging with develop
  changes for dryrun
  fixed type depecendies.repos
  added position sequence
  initial mini coordinator commit
  fixed missing/wrong packages
  added move absolute action service
* rempved mini_coordinator
* added position sequence
* initial mini coordinator commit
* added meshes
* fixed missing/wrong packages
* Pull request #8: Feature/drill pattern BT
  Merge in BAUTIRO/fpm from feature/drill_pattern_BT to develop
  * commit '5ab26f888c71cdcea4adfef9883cfecda6829b3f': (27 commits)
  added lift feedback and waiting for target achieved
  updated licence
  fixed coordinates for moving tcp in workplane
  switched to joint pose for configured poses
  Moveit is working for the complete bautiro modell
  adding again lift controller launch file
  moveit works again !
  merged ur description
  before major merge process for complete bautiro and moveit integration
  updated srdf files for complete bautiro description
  added moveit configuration
  WIP modification according merged bautiro description
  Adding transform listener for lift
  integrated control publisher for the action node of the lift
  adding lift command service
  renaming
  fixed error in pose and orientation mapping
  working example for move relative, and configured pose
  added reading parameters
  renaming into fpm_cu
  ...
* updated licence
* switched to joint pose for configured poses
* added reading parameters
* moved move_group example to the fpm_demo repo
* added docu and modified cartesian positions
* added action server and geometry publisher
* added launch file for fpm_cu action server
* Pull request #6: Feature/moveit configuration
  Merge in BAUTIRO/fpm from feature/moveit_configuration to develop
  * commit '311a0ec1f40842c13bf96ade6193d538a08e584d':
  moved ur_sim_controller
  removed build errors
  README, package xml update
  clean up
  clean up of unused files
  moveit working example
  removed panda stuff from rviz config file
  WIP removed launch file erros, still not working
  changed name to exclude control manager
  gazebo model spawn, hardware interfaces available, controller could be loaded
  adding simulation launch file for fpm_description
  Added moveit config and ur16 description
  copied moveit configuration from ur_moviet_config
* removed build errors
* Added moveit config and ur16 description
* copied moveit configuration from ur_moviet_config
* Pull request #5: Feature/move sim demo files
  Merge in BAUTIRO/fpm from feature/move_sim_demo_files to develop
  * commit '6be1ad2e33b7df2c3023b7e90187d5b46e3fce81':
  naming and docu
  update repo readme  and docu
  Readme removed MoveIT package
  moved demo files
  moved fpm_gazebo_simulation package
* naming and docu
* update repo readme  and docu
* moved demo files
* Pull request #4: Feature/add ur16e
  Merge in BAUTIRO/fpm from feature/add_ur16e to develop
  * commit '1d2d9e186f71b2623feb98a0cbb3f9316afe97a3':
  added according PR meeting
  remapping controller msgs
  added launch file for cyclic trajectory publisher
  added fpm demo package
  added demo positions
  added comments to urdf.xacro
  separated the xacro robot description
  added dummy ros2 control yaml file
* added according PR meeting
* remapping controller msgs
* added launch file for cyclic trajectory publisher
* added fpm demo package
* Contributors: Andreas Mogck, Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Premkumar Raamkishore (PT/PJ-TOP100), Rothacker Elisa (CR/AAS4), Shaikh Mohammed Nawaz (PT/PJ-TOP100), Sinisa Slavnic(CR/APT5), mia4si, prr1le, shm3le
