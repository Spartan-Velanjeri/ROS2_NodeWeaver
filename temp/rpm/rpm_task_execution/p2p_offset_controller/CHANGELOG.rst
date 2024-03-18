^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package p2p_offset_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #7: Feature/BAUTIRO-428 preparation autonomous driving for november demo with fus1
  Merge in BAUTIRO/rpm_task_execution from feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1 to develop
  * commit '5e8b6e5606a1880d94cb1e239e6c6f2a72accacf':
  commited by ngq3fe: change in behaviour tree node to increase stability, do not use lookup transform for static parameter
  ngq3fe: preparation NovDemo
  behaviour tree with switch point and tested p2p controller
  added controller gain and comments
  commited by ngq3fe: reduce limits
  roslogging statements
  add calculation_switch_point behaviour tree action node
  fixed scaling
  added scaling
  commited by ngq3fe: controller tested on RPM2
  p2p controller for RPM1 and RPM2
  modify p2p controller for RPM2
* ngq3fe: preparation NovDemo
* behaviour tree with switch point and tested p2p controller
* added controller gain and comments
* commited by ngq3fe: reduce limits
* add calculation_switch_point behaviour tree action node
* fixed scaling
* added scaling
* commited by ngq3fe: controller tested on RPM2
* Pull request #6: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/rpm_task_execution from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit 'aeaee43b0e4e16b95760ac0aada726131ca606b3':
  Update Jenkinsfile
  Add build status badge
  Fix flake8 issues
  Disable testing for now
  Fix typo with declaration of visualization_msgs
* p2p controller for RPM1 and RPM2
* modify p2p controller for RPM2
* Fix flake8 issues
* removed  dependency
* Pull request #3: remove deleted file from install targets
  Merge in BAUTIRO/rpm_task_execution from bugfix/BAUTIRO-258-fix-build-error-in-rpm_tast_execution to develop
  * commit '186344e66966d3d947911a2cf4239f19f91fe303':
  remove deleted file from install targets
* remove deleted file from install targets
* remove testing file
* Pull request #2: Feature/BAUTIRO-182 call data service for cluster hole in ccu mission execution
  Merge in BAUTIRO/rpm_task_execution from feature/BAUTIRO-182-call-data-service-for-cluster-hole-in-ccu-mission-execution to develop
  * commit '7c05dfbb6d9fad5f202f6a14b6b25de8434a894f':
  handle empty bt_xml input
  update launch file name
  update action name and parameters
  correct marker position to ur_base instead of base_link
  add goal pose marker for rviz
  cleanup transform node
  working offset alogorithm
  fix all port name mismatches
  fix xml port name mismatch
  fix interface parameters naming issue
  revert interfaces
  update to modified interface
  add offset node in bt to offset goal pose
* add offset node in bt to offset goal pose
* Pull request #1: BAUTIRO-108 implement ccu fpm rpm task execution structure
  Merge in BAUTIRO/rpm_task_execution from BAUTIRO-108-implement-ccu-fpm-rpm-task-execution-structure to develop
  * commit 'eff476320d1ef9f2cad966803957c1df410cd635':
  update motion to nav2 instead of p2p and fix bugs
  fix namespace and goal to bb issues
  update xml and bb parameters
  building rpm bt pkgs and p2p controller
* building rpm bt pkgs and p2p controller
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Marcusso Manhaes Musa Morena (CR/AAS3), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro on rpm2
