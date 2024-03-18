^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm into feature/BAUTIRO-518-simulation-mode-for-ts16
* Pull request #25: Feature/BAUTIRO-378 test in schillerhoehe
  Merge in BAUTIRO/fpm from feature/BAUTIRO-378-test-in-schillerhoehe to develop
  * commit 'e2978608f167da12c957de3c939c314cabf3f180':
  small typo fix
  commited by ngq3fe: changes in launch file
  commited by ngq3fe: change on sh construction site
  fix: removing namespace
  feat: Launching changes * Tmux for hu (order of launching is important) * Lauch change
* fix: removing namespace
* feat: Launching changes
  * Tmux for hu (order of launching is important)
  * Lauch change
* Pull request #23: BAUTIRO-50 bringup files
  Merge in BAUTIRO/fpm from BAUTIRO-50-bringup-files to develop
  * commit 'a64fe2d13b5ee8f43cea51f11ccd86ce7db55bc0':
  fix: version revert.
  feat: Launch for additional fpm modules.
  feat: Node parameters.
  feat: Namespace with GroupAction.
  feat: Namespaces Namespaces added to launch nodes. Separate launch of vacuum and lift.
  feat: Permission change
  feat: added rules for usb devices
  Initial test of bringup files.
* fix: version revert.
* feat: Launch for additional fpm modules.
* feat: Node parameters.
* feat: Namespace with GroupAction.
* feat: Namespaces
  Namespaces added to launch nodes.
  Separate launch of vacuum and lift.
* Initial test of bringup files.
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
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm into prr1le/testing
* add directory `fpm_bringup/launch` to fix builds
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
* final changes before merging with develop
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
* added docu files
* initial commit
* Contributors: Lukas Kiefer DC/PAR, Mogck Andreas (CR/AAS5), Nguyen Quang Huy (CR/AAS5), Rothacker Elisa (CR/AAS4), SSL2LR, Schumacher Georg (PT/PJ-TOP100), Sinisa Slavnic (CR/APT5), Sinisa Slavnic(CR/APT5), Slavnic Sinisa (CR/APT5), mia4si
