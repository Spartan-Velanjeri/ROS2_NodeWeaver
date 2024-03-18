^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package handling_unit_motion_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #20: BAUTIRO-791 documentation of the hu motion manager
  Merge in BAUTIRO/fpm_motion_manager from BAUTIRO-791-documentation-of-the-hu-motion-manager to develop
  * commit '54f873e73754c9b6cee8646e988193ffbcba6a81':
  feat: skills description.
  docs: ReadMe for hu motion manager.
* docs: ReadMe for hu motion manager.
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
* added dummy program for test call of urp
* Merge branch 'feature/BAUTIRO-337' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm_motion_manager into feature/BAUTIRO-362
* Pull request #12: Feature/BAUTIRO-337
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-337 to develop
  * commit 'c0f784398bdd05fd05bc4497e241bb8c1a24d0a7':
  program moveit <-> drilling protoype
  adding logger info for desired pose
  set velocity scaling
  updated relative movement print
  code cleanup
  minor clean ups
* adding logger info for desired pose
* set velocity scaling
* updated relative movement print
* code cleanup
* minor clean ups
* Pull request #10: Feature/BAUTIRO-295
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-295 to develop
  * commit '09f6ba5a655d6535fb0624ec2516272fd2e87094':
  comment for array order
  added comment for importing array from ur script
  updated default positions
* comment for array order
* added comment for importing array from ur script
* updated default positions
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
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm_motion_manager into feature/BAUTIRO-277
* Pull request #9: Feature/BAUTIRO-183 develop branch for fpm motion manager
  Merge in BAUTIRO/fpm_motion_manager from feature/BAUTIRO-183-develop-branch-for-fpm_motion_manager to develop
  * commit 'e61d0493c514f8792de331d70ed14b7296ad6f42':
  changes for hu_ur_6axis
  updated launch file robot description
  renamed to fpm_tree_nodes again
  section for fpm motion manager
  changes fore launcg files
  Succesfull build after renaming
  added drawing
  notes modified
  adding notes for pose discussion with volker
  reractoring and discussion with Volker
* changes for hu_ur_6axis
* Merge branch 'feature/BAUTIRO-183-develop-branch-for-fpm_motion_manager' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/fpm_motion_manager into feature/BAUTIRO-183-develop-branch-for-fpm_motion_manager
* updated launch file robot description
* section for fpm motion manager
* changes fore launcg files
* Succesfull build after renaming
* reractoring and discussion with Volker
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Andreas Mogck, Mogck Andreas (CR/AAS5), Nguyen Quang Huy (CR/AAS5), Sinisa Slavnic (CR/APA3), Slavnic Sinisa (CR/APT5), bautiro
