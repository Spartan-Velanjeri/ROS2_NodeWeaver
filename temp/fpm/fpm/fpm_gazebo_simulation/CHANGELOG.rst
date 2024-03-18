^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_gazebo_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* urdf changes
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
* moveit works again !
* merged ur description
* before major merge process for complete bautiro and moveit integration
* updated srdf files for complete bautiro description
* added moveit configuration
* WIP modification according merged bautiro description
* Merge branch 'develop' into feature/drill_pattern_BT
* Pull request #7: Feature/merge robot model
  Merge in BAUTIRO/fpm from feature/merge_robot_model to develop
  * commit '4d877d513e6c4b3d0a1c44bb443e42342889e526':
  launch file configured to run plugin through launch file
  added todo comment
  restructre of controller, gazebo plugins and launch files
* restructre of controller, gazebo plugins and launch files
* added reading parameters
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
* clean up of unused files
* moveit working example
* WIP removed launch file erros, still not working
* changed name to exclude control manager
* gazebo model spawn, hardware interfaces available, controller could be loaded
* adding simulation launch file for fpm_description
* Added moveit config and ur16 description
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
* Readme removed MoveIT package
* moved fpm_gazebo_simulation package
* Contributors: Mogck Andreas (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Sinisa Slavnic(CR/APT5), mia4si
