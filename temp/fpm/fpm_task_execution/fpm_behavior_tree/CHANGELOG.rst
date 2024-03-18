^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_behavior_tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #10: removed obsolete behavior tree plugins
  Merge in BAUTIRO/fpm_task_execution from feature/BAUTIRO-573 to develop
  * commit 'd38cb1196df77139ba5b0dd3ad7b5b6d812db44f':
  removed obsolete behavior tree plugins
* removed obsolete behavior tree plugins
* Pull request #7: Feature/BAUTIRO-378 sw tests in schillerhoehe
  Merge in BAUTIRO/fpm_task_execution from feature/BAUTIRO-378-sw-tests-in-schillerhoehe to develop
  * commit 'df626a16a7c616e38e2e24c60cc797deb04c53ad': (27 commits)
  mia4si
  mia4si: print target in leice coordinates
  mia4si: lu bt available at skill_server
  changes for transform clients
  merged all lu clients into one library
  committed by ngq3fe: some change in fein localization
  sort duplicates and changes for ROS2 control
  sorted hu clients
  name changes for behavior tree
  modified tree for drilling pattern
  fix: SSL2LR Bug fix
  feat: New action for relative motion of the robot. * HUMoveRelative is not functional.
  feat: actions for hu
  feat: BT modified * Any skill can be started and error will not be returned.
  commited by ngq3fe: changes for test lu_fpm.xml on schillerhöhe
  restore changes from last commit
  commited by ngq3fe: changes on sh construction site
  fix: fixing build failure
  fix: change of topic  and manual drill program
  feat: manual drilling * reverting plug-in inclusion from hu_motion_manager * typo in variable name * hu actions added
  ...
* mia4si: lu bt available at skill_server
* merged all lu clients into one library
* Merge branch 'feature/BAUTIRO-378-sw-tests-in-schillerhoehe' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm_task_execution into feature/BAUTIRO-378-sw-tests-in-schillerhoehe
* committed by ngq3fe: some change in fein localization
* sort duplicates and changes for ROS2 control
* sorted hu clients
* name changes for behavior tree
* modified tree for drilling pattern
* feat: New action for relative motion of the robot.
  * HUMoveRelative is not functional.
* feat: BT modified
  * Any skill can be started and error will not be returned.
* commited by ngq3fe: changes for test lu_fpm.xml on schillerhöhe
* restore changes from last commit
* commited by ngq3fe: changes on sh construction site
* fix: fixing build failure
* Pull request #6: BAUTIRO-21 drilling according test case 351
  Merge in BAUTIRO/fpm_task_execution from BAUTIRO-21-drilling-according-test-case-351 to feature/BAUTIRO-378-sw-tests-in-schillerhoehe
  * commit 'e89d583cf59645c40fd232e0bda856d4f95dfbac':
  fix: change of topic  and manual drill program
  feat: manual drilling * reverting plug-in inclusion from hu_motion_manager * typo in variable name * hu actions added
  feat: manuall drill
  feat: manuall drill
  feat: Included motions to the configured poses.
  feat: reading of lift position.
  feat: fpm drill manually skill.
  feat: Lift actions. * Lift move absolute moved to lift_action.cpp * Lift move to working height added.
  feat: Skills to move lift and robot.
* fix: change of topic  and manual drill program
* feat: manual drilling
  * reverting plug-in inclusion from hu_motion_manager
  * typo in variable name
  * hu actions added
* feat: manuall drill
* feat: manuall drill
* feat: Included motions to the configured poses.
* feat: fpm drill manually skill.
* feat: Lift actions.
  * Lift move absolute moved to lift_action.cpp
  * Lift move to working height added.
* Pull request #5: Feature/BAUTIRO-226 transform drill holes from map frame to ur base
  Merge in BAUTIRO/fpm_task_execution from feature/BAUTIRO-226-transform-drill-holes-from-map-frame-to-ur-base to develop
  * commit 'b5e68ba28ff6ac584d4de3649f11f591f722b4fc':
  cleanup
  fix wrong launch file name
  handle empty bt_xml input
  remove transform node from sim bt xml
  update launch file name
  add comments to drillhole tranform node
  add alternative transformation technique
  fix runtime errors
  resolve port name mismatch
  fix run time errors
  add node in xml
  add drill hole pose transformation node
* feat: Skills to move lift and robot.
* cleanup
* fix wrong launch file name
* fix typo
* handle empty bt_xml goal
* remove old interface and client
* Pull request #4: Feature/BAUTIRO-182 call data service for cluster hole in ccu mission execution
  Merge in BAUTIRO/fpm_task_execution from feature/BAUTIRO-182-call-data-service-for-cluster-hole-in-ccu-mission-execution to develop
  * commit 'd00caf2baf6a3c9d17280910cf096862cbbf84b7':
  add number of drill masks through action call instead of node
  update to modified interface and remove redundancy
* handle empty bt_xml input
* remove transform node from sim bt xml
* update launch file name
* resolve port name mismatch
* fix run time errors
* add node in xml
* add drill hole pose transformation node
* add number of drill masks through action call instead of node
* update to modified interface and remove redundancy
* restructure with tasks folder
* Merge branch 'feature/BAUTIRO-108-implement-ccu-fpm-rpm-task-execution-structure' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/fpm_task_execution into develop
* both trees visualizable
* two trees launching, but only can be visualized in groot
* bt_navigation.cpp edited online with Bitbucket
  remove unavailable and unused node
* Pull request #1: Feature/BAUTIRO-108 implement ccu fpm rpm task execution structure
  Merge in BAUTIRO/fpm_task_execution from feature/BAUTIRO-108-implement-ccu-fpm-rpm-task-execution-structure to develop
  * commit 'bf293f0b196d9d4fd692c9a7c81921cf00435130':
  Commited by aba1le: Working May Demo Behavior Tree, tested with FUS1
  fix pose msg mismatch
  a
  fix not working issues
  Revert "update folder structure and rename bt_nav content"
  updated structure
  restructured
  update folder structure and rename bt_nav content
  fix runtime errors due to mismatched msg types
  update launch file
  fix build issue
  add missing request inputs
  create pkg, bt, plugins
  create generic xml
  update namespaces in code
  create bt packages
* Commited by aba1le: Working May Demo Behavior Tree, tested with FUS1
* fix pose msg mismatch
* a
* Revert "update folder structure and rename bt_nav content"
  This reverts commit 5452e834d6f0f91ed4bed2c519b5f22dfaa13759.
* updated structure
* restructured
* update folder structure and rename bt_nav content
* fix runtime errors due to mismatched msg types
* update launch file
* create pkg, bt, plugins
* create generic xml
* update namespaces in code
* create bt packages
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Mogck Andreas (CR/AAS5), Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Sinisa Slavnic(CR/APT5), Slavnic Sinisa (CR/APT5), bautiro, bautiro from rpm
