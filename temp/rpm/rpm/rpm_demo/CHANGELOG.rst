^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rpm_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Fix version
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1
* Pull request #30: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/rpm from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '5ecc3dbaae5eb27a14bd17f34d0e55efc13738d3':
  Update Jenkinsfile
  Reformat some XML files
  Remove ament_python as build_type
  Use ament_cmake_python instead of ament_python
  Use feature branch for rpm_powertrain_driver
  Add ros2_canopen dependency
  Add missing rpm_powertrain_driver dependency
  Build geographic_transform from master
  Add geographic_transform
  Add ros_compat
  Try to build with upstream
  Add build status badge
  Add Jenkinsfile
* Remove ament_python as build_type
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/rpm into BAUTIRO-15-create-rpm-launch-files
* Pull request #20: Feature/BAUTIRO-278
  Merge in BAUTIRO/rpm from feature/BAUTIRO-278 to develop
  * commit '146645580c4183aea376f74b62802d57e45966f6':
  add velocity calculator to rpm_demo
  remove unnecessary collision bodies
  commited by erz2lr: changes during running up SLAM on FUS1 and final robot description
  delete joint from base_link to lift_base_mount in rpm_chassis
  changes in agruments
  changed upper cover back to normal collision
  changed footprint sized
  fixed rpm_localization
  collision for upper cover changed to box
  changes to fit robot model to FUS1
  commited by ngq3fe: changes for 278
  commited by ngq3fe: changes for 278
  Added new maps and solved build for rpm_localization
* Pull request #19: add velocity calculator to rpm_demo
  Merge in BAUTIRO/rpm from BAUTIRO-293-calculation-of-current-velocity to feature/BAUTIRO-278
  * commit 'b3d26c8488d1d4b7618811cc0559b3f80ecb953b':
  add velocity calculator to rpm_demo
* add velocity calculator to rpm_demo
* Pull request #2: rpm_demo package added with cyclic Twist Msg Publisher
  Merge in BAUTIRO/rpm from feature/configure_ros2_control to develop
  * commit '85ec295f0cc75800103a2c38acb28688c6c84a91':
  rpm_demo package added with cyclic Twist Msg Publisher
* rpm_demo package added with cyclic Twist Msg Publisher
* Contributors: Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Schumacher Georg (PT/PJ-TOP100)
