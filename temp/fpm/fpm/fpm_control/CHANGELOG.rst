^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
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
* modified for lift controller
* updated parameters for lift controller 3 segment
* Pull request #11: Bulk movement from testing branch to develop branch
  Merge in BAUTIRO/fpm from prr1le/testing to develop
  * commit '170685de72e73523030fc2216a94b73cb34f0a38':
  changes according review
  changed forward to lift command controller
  added hollow cad models to implement collision in lift
  added joints and new cad model
  added meshes
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into prr1le/testing
* changed forward to lift command controller
* added joints and new cad model
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
* adding again lift controller launch file
* moveit works again !
* merged ur description
* before major merge process for complete bautiro and moveit integration
* added moveit configuration
* Merge branch 'develop' into feature/drill_pattern_BT
* Pull request #7: Feature/merge robot model
  Merge in BAUTIRO/fpm from feature/merge_robot_model to develop
  * commit '4d877d513e6c4b3d0a1c44bb443e42342889e526':
  launch file configured to run plugin through launch file
  added todo comment
  restructre of controller, gazebo plugins and launch files
* restructre of controller, gazebo plugins and launch files
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
* WIP removed launch file erros, still not working
* removed launch dir from fpm_control package
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
* added demo positions
* added dummy ros2 control yaml file
* added ros run cmd line for sending commands to the documentation
* removed misnamed launch file
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into develop
* added readme with how to run
* Pull request #2: Added test node to send cyclic commands to fwd controller
  Merge in BAUTIRO/fpm from feature/fwd_controller_test to develop
  * commit 'bc6fef7aa8e6b99591362a793958ae83fa0db78c':
  added dependencies
  added launch files for test node
  added visualization
  initial commit
* added dependencies
* added launch files for test node
* added visualization
* initial commit
* Pull request #1: Adding lifting unit
  Merge in BAUTIRO/fpm from adding-lifting-unit to develop
  * commit '642a929bc81944f3d442d813663a0c35a5d1b568':
  fixed frame for viewing lifting unit
  fixed wrong offset fot lift top
  modified control config
  fixed yaml name error in urdf description
  robot description ready for rviz
  added docu files
  initial commit fpm urdf
  modified effort.yaml
* modified control config
* fixed yaml name error in urdf description
* robot description ready for rviz
* modified effort.yaml
* adding controller configs for lifting unit
* WPM added contrl package for UR
* Contributors: Andreas Mogck, Mogck Andreas (CR/AAS5), Premkumar Raamkishore (PT/PJ-TOP100), Schumacher Georg (PT/PJ-TOP100), Sinisa Slavnic(CR/APT5), mia4si, prr1le
