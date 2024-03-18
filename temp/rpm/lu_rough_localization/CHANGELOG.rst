^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lu_rough_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #2: Feature/BAUTIRO-428 preparation autonomous driving for november demo with fus1
  Merge in BAUTIRO/lu_rough_localization from feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1 to develop
  * commit 'fb9910a0a7ab7ddd20e864136533b1b22209ec7a':
  correct comments
  add SH map shifted to zero
  Implemented spatial, temporal and combined spatial-temporal filtering
  Combined spatial and temporal filter
  Add temporal filtering
  Clean up
  erz2lr: tests mit FUS2 16.11.2023
  bug fix byte shift
  erz2lr: tests am FUS3
  filter_outliers in PCD
* correct comments
* add SH map shifted to zero
* Implemented spatial, temporal and combined spatial-temporal filtering
* Combined spatial and temporal filter
* Add temporal filtering
* Clean up
* erz2lr: tests mit FUS2 16.11.2023
* Merge branch 'feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/lu_rough_localization into feature/BAUTIRO-428-preparation-autonomous-driving-for-november-demo-with-fus1
* bug fix byte shift
* erz2lr: tests am FUS3
* filter_outliers in PCD
* Pull request #1: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/lu_rough_localization from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '6534633f10ecfcc4426d7cd426d37e8965f0158e':
  Update Jenkinsfile
  Remove ament_python as builddepend_tool
  Add license
  Add build status badge
  Disable testing for now
  Fix filename
* Update Jenkinsfile
* change topics name
* Remove ament_python as builddepend_tool
* Add license
* Add build status badge
* Disable testing for now
* Fix filename
* add bag map for construction site in schillerhoehe
* solve merge conflict
* maps/rosbag_shroom1_externed_no_bar
* maps/rosbag_shroom1_externed_cropped
* commited by erz2lr@fus1: transform service and manipulate yaml for slam
* commited by ngq3fe: add from obj generated map of le131
* correct frame_id
* Add exception handling
* clean up
* test avalaibility of robot description
* add  map rosbag_shroom1
* move rpm_localization nodes to lu_rough_localization
* correct colcon build errors
* Initial Commit
* Contributors: Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), bautiro from rpm, bautiro on rpm2
