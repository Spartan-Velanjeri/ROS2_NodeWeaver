^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hu_behavior_tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #20: BAUTIRO-791 documentation of the hu motion manager
  Merge in BAUTIRO/fpm_motion_manager from BAUTIRO-791-documentation-of-the-hu-motion-manager to develop
  * commit '54f873e73754c9b6cee8646e988193ffbcba6a81':
  feat: skills description.
  docs: ReadMe for hu motion manager.
* feat: skills description.
* Pull request #15: Feature/BAUTIRO-378 test in schillerhoehe
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-378-test-in-schillerhoehe to develop
  * commit 'd6b7738cf2049a908425bfe015ff4948f6f03dc5':
  added tf2 for transforming goals
  mia4si: added init behavir hu_behavior_tree
  added dummy program for test call of urp
  added ros2 control bt
  call urp programm without drilling
  added move relative action client
  feat: Change of the drill program (SSL2LR)
  feat: Transport in-out skills.
* added tf2 for transforming goals
* mia4si: added init behavir hu_behavior_tree
* added dummy program for test call of urp
* added ros2 control bt
* call urp programm without drilling
* feat: Change of the drill program (SSL2LR)
* feat: Transport in-out skills.
* Pull request #14: Feature/BAUTIRO-362
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-362 to develop
  * commit 'a2024018f93ed5f0c989c3d5a2b36e70b35880e1':
  added return and behavior tree for execute URP
  added two asynchronous nodes checking io state and program status
* Merge branch 'feature/BAUTIRO-337' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm_motion_manager into feature/BAUTIRO-362
* added return and behavior tree for execute URP
* Pull request #12: Feature/BAUTIRO-337
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-337 to develop
  * commit 'c0f784398bdd05fd05bc4497e241bb8c1a24d0a7':
  program moveit <-> drilling protoype
  adding logger info for desired pose
  set velocity scaling
  updated relative movement print
  code cleanup
  minor clean ups
* program moveit <-> drilling protoype
* Pull request #11: build was broken, working again
  Merge in BAUTIRO/fpm_motion_manager from hotfix/update_package_xml to develop
  * commit '2b1384897ab5c49f8f9d13c38ada8ba720a03191':
  build was broken, working again
* build was broken, working again
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
* add tmux script and fix minor issues to move to move arm in ursim
* working behavior tree without tested move it due to infeasible pose
* resolve mismatched factory names
* remove fpm_handling_unit_bt and add new moveit_program_control xml
* Remove LU clients, one cpp file for motion manager clients, Move cluster components to fpm_cu, one cpp file for point handling
* update to new interface
* resolve load_points to work with data_services and yaml file
* handle empty bt_xml and test 2 more xmls
* working dashboard_bt xml
* create hu bt pkgs, remove cluster_interfaces and update cluster_components with baurito_ros_interfaces
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Mogck Andreas (CR/AAS5), Nguyen Quang Huy (CR/AAS5), Sinisa Slavnic(CR/APT5), Slavnic Sinisa (CR/APT5), bautiro
