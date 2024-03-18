^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fpm_handling_unit_script_switching
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #16: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/fpm_motion_manager from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit '13f1e15f54ef7eda4c48dfa78bff0d10cd33206d':
  Add build status badge
  Source local profile
  Fix typo
* Fix typo
* Pull request #8: Feature/BAUTIRO-277 sort behaviortree of fpm motion manager
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-277-sort-behaviortree-of-fpm_motion_manager to develop
  * commit '979b8d9185b9dec1479e317d34487e0626c3dde2':
  add tmux script and fix minor issues to move to move arm in ursim
  working behavior tree without tested move it due to infeasible pose
  resolve mismatched factory names
  fix program not terminating
  delete load_points_client file after adding to points_handler
  remove fpm_handling_unit_bt and add new moveit_program_control xml
  cleanup
  Remove LU clients, one cpp file for motion manager clients, Move cluster components to fpm_cu, one cpp file for point handling
  cleanup
  update to new interface
  resolve load_points to work with data_services and yaml file
  handle empty bt_xml and test 2 more xmls
  working dashboard_bt xml
  create hu bt pkgs, remove cluster_interfaces and update cluster_components with baurito_ros_interfaces
* Remove LU clients, one cpp file for motion manager clients, Move cluster components to fpm_cu, one cpp file for point handling
* cleanup
* create hu bt pkgs, remove cluster_interfaces and update cluster_components with baurito_ros_interfaces
* fixed build errors
* Pull request #6: Feature/script switching
  Merge in BAUTIRO/fpm_motion_manager from feature/script_switching to develop
  * commit '061496b5bc79463844edcccc8c7301341c0ed0d9':
  added action client for fpm drill cluster tree
  added fpm bt action server
  added fpm bt action server
* added fpm bt action server
* Pull request #4: Feature/script switching
  Merge in BAUTIRO/fpm_motion_manager from feature/script_switching to develop
  * commit '8267366a6a2bb36df404f4fda6ed4e6445350f97':
  deleted fpm motion server package for docker build
  removed some lines in cmake
  removed some lines in cmake
  added files for drill cluster
  added files for drill cluster
  added files for drill cluster
  included behavior tree files for testbench
  included behavior tree files for testbench
  included behavior tree files for testbench
  included behavior tree files for testbench
  included behavior tree files for testbench
  included behavior tree files for the testbench
* removed some lines in cmake
* added files for drill cluster
* added files for drill cluster
* included behavior tree files for testbench
* included behavior tree files for testbench
* included behavior tree files for testbench
* included behavior tree files for testbench
* included behavior tree files for the testbench
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Avula Praneeth (PT/PJ-TOP100), Marcusso Manhaes Musa Morena (CR/AAS3), Mogck Andreas (CR/AAS5), Musa Morena Marcusso Manhaes, avp1le
