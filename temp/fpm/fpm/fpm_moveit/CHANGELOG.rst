^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #28: BAUTIRO-668 documentation for the moveit package
  Merge in BAUTIRO/fpm from BAUTIRO-668-documentation-for-the-moveit-package to develop
  * commit 'aaa1100e4cf02860bd66612ed212a5c6f81ef6f9':
  docs: Calibration docu.
  docs: test links.
  docs: Description of the config files
  docs: Configuration explained.
  docs: Test once more.
  docs: Checkis html in repo fine?
* docs: Calibration docu.
* docs: test links.
* docs: Description of the config files
* docs: Configuration explained.
* docs: Test once more.
* docs: Checkis html in repo fine?
* Merge branch 'develop' into BAUTIRO-50-bringup-files
* Pull request #21: change for servoing function
  Merge in BAUTIRO/fpm from feature/BAUTIRO-337 to develop
  * commit '18d6e87b7e5e8cc8be7738d78facfda2cec08f6a':
  change for servoing function
* change for servoing function
* Pull request #20: Feature/BAUTIRO-294
  Merge in BAUTIRO/fpm from feature/BAUTIRO-294 to develop
  * commit '2d9675bd9bbff9df4c845d7825902de496f4630d':
  using drill bit position as tip link
  added ptu drill bit tcp
  adding stl and link to xacro
* using drill bit position as tip link
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
* planning with RVIZ is working
* adapted velocity and tested joint poses on FUS1
* added inital poses
* added launch file for rviz
* undo Launchargument for load_yaml
* changes for changes in move_group name
* modifications for prefix
* Pull request #14: Feature/staging develop
  Merge in BAUTIRO/fpm from feature/staging_develop to develop
  * commit 'cdc193a6475735dfd4195784bace9010dfedc797': (24 commits)
  change ssh to https in dependencies.repos
  created .repos file with maydemo2023 tag
  mods for  planning and collision avoidance FUS1
  set height to 2.4m matching real lift
  modified for lift controller
  added nav2
  added calibration informations to launch file
  changed branch after merged pull request
  added groot to dependencies.repos
  changed .repos
  removed due to renaming
  added behavior tree
  fixed type
  fixed ros2 controller
  changes for lift urdf and controller
  updated parameters for lift controller 3 segment
  adding develop lift version
  robot desc with calibration
  add moveit robot driver etc
  update .repos
  ...
* mods for  planning and collision avoidance FUS1
* added calibration informations to launch file
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
* added position sequence
* added meshes
* fixed missing/wrong packages
* remove hu\_* dependencies
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
* Moveit is working for the complete bautiro modell
* moveit works again !
* merged ur description
* before major merge process for complete bautiro and moveit integration
* updated srdf files for complete bautiro description
* added moveit configuration
* WIP modification according merged bautiro description
* added docu and modified cartesian positions
* added launch file for fpm_cu action server
* initial commit for fpm action server
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
* moved ur_sim_controller
* removed build errors
* README, package xml update
* clean up
* clean up of unused files
* moveit working example
* removed panda stuff from rviz config file
* WIP removed launch file erros, still not working
* Added moveit config and ur16 description
* copied moveit configuration from ur_moviet_config
* Contributors: Andreas Mogck, Kiefer Lukas (DC/PAR), Mogck Andreas (CR/AAS5), Sinisa Slavnic (CR/APA3), Sinisa Slavnic(CR/APT5), Slavnic Sinisa (CR/APT5), mia4si, prr1le
