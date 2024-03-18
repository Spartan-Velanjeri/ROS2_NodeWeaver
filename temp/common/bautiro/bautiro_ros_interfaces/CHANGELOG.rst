^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bautiro_ros_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #40: Define fine_localization service
  Merge in BAUTIRO/bautiro_common from BAUTIRO-558-remove-duplicate-bt-clients-for-lu to develop
  * commit '49b9a1440a0d25203673d134505427b59f6dc1bb':
  Remove LuFineLocalization.srv from CMakeLists.txt
  Moved LuFineLocalization.srv to fpm/src/fpm/fpm_interfaces/srv --> BAUTIRO-499
  Define fine_localization service
* Remove LuFineLocalization.srv from CMakeLists.txt
* Moved LuFineLocalization.srv to fpm/src/fpm/fpm_interfaces/srv --> BAUTIRO-499
* Define fine_localization service
* Pull request #37: Feature/BAUTIRO-80 ccu backend for obj file transfer MERGED
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-80-ccu-backend-for-obj-file-transfer_MERGED to develop
  * commit 'f7f14f60b33a960f4023d176e0759380aabac1cc':
  added drill mask for manual mode to be available for skills
  added marker and pattern for manual mode
  BAUTIRO-80 GenericPrim.srv for trivial Interfaces
  Add hash to MissionDataResponse and MissionMetaData
  align hcu-ccu interface with taskplan
  fix filecount type
  rename RoomPlanFileMetaData and RoomPlanFileTransferCompletedMessage
  BAUTIRO-486  - add Drill command
  proto definitions for file transfer
* added drill mask for manual mode to be available for skills
* added marker and pattern for manual mode
* Merge remote-tracking branch 'origin/develop' into test
* Pull request #35: BAUTIRO-378 sw tests in schillerhoehe
  Merge in BAUTIRO/bautiro_common from BAUTIRO-378-sw-tests-in-schillerhoehe to develop
  * commit 'b8ab07d77a8fad2ef70ccb89dce010410a1491ed':
  feat: additional actions. * Fpm actions * Typo changes
  feat: action name change
* BAUTIRO-80 GenericPrim.srv for trivial Interfaces
  used in ccu_hcu_abstraction <--> ccu_data_services
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro_common into BAUTIRO-378-sw-tests-in-schillerhoehe
* Pull request #30: BAUTIRO-217 Propagate marker info by data service
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-216-update-data-service-and-behavior-tree-to-send-also-marker-information-for-fine to develop
  * commit 'f2e48008041423e50729153096306cc1b7b6782f':
  BAUTIRO-217 Propagate marker info by data service
* Merge branch 'origin/develop' into
  feature/BAUTIRO-216-update-data-service-and-behavior-tree-to-send-also-marker-information-for-fine
* Pull request #33: BAUTIRO-21 drilling according test case 351
  Merge in BAUTIRO/bautiro_common from BAUTIRO-21-drilling-according-test-case-351 to BAUTIRO-378-sw-tests-in-schillerhoehe
  * commit 'fe50009ec40fe351fea4f7ba241fd733262d119a':
  feat: additional actions. * Fpm actions * Typo changes
  feat: action name change
* feat: additional actions.
  * Fpm actions
  * Typo changes
* feat: action name change
* Pull request #32: BAUTIRO-50 bringup files
  Merge in BAUTIRO/bautiro_common from BAUTIRO-50-bringup-files to develop
  * commit '35a61bdf64b343c210f3805075060d8e2b1ef30b':
  feat: Lift service and hu action.
  feat: Fpm actions for HU.
  feat: Action for FPM.
  fix: missing file
  feat: coordinator actions
* feat: Lift service and hu action.
* Merge remote-tracking branch 'origin/develop' into BAUTIRO-50-bringup-files
* Merge branch 'develop' into BAUTIRO-50-bringup-files
* feat: Fpm actions for HU.
* feat: Action for FPM.
* Pull request #31: removed typo
  Merge in BAUTIRO/bautiro_common from hotfix/fix-interface-error to develop
  * commit '56478416ad9d5107e4d05e7da4111321c704982d':
  removed typo
* removed typo
* Pull request #29: adding ENUMs for configured poses
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-337 to develop
  * commit 'aec281f454be2f0c7ba2cc747e49d55dcf8ae4d0':
  removed comments, no longer valid
  Proper naming for suffix
  adding ENUMs for configured poses
* removed comments, no longer valid
* BAUTIRO-217 Propagate marker info by data service
* Proper naming for suffix
* adding ENUMs for configured poses
* fix: missing file
* feat: coordinator actions
* Pull request #28: add new action for hu behavior tree
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-277-sort-behaviortree-of-fpm_motion_manager to develop
  * commit '0333bead725934b25b7088f1a292d18d9f5505e4':
  add new action for hu behavior tree
* add new action for hu behavior tree
* Pull request #26: fix: Added deleted definitions.
  Merge in BAUTIRO/bautiro_common from BAUTIRO-50-bringup-files to develop
  * commit '6a8089dfd6f105bffc6708ae5495081d2b6767f2':
  fix: Added deleted definitions.
* fix: Added deleted definitions.
* Pull request #23: BAUTIRO-225 added ProtoInRos.srv for string encapsulation of Protobuf(grpc).msg in string(ROS).msg
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-225 to develop
  * commit 'c2ed7909f020c989d6793d503c3ecce4c15248af':
  BAUTIRO-225 added ProtoInRos.srv for string encapsulation of Protobuf(grpc).msg in string(ROS).msg
* BAUTIRO-225 added ProtoInRos.srv for string encapsulation of Protobuf(grpc).msg in string(ROS).msg
* Pull request #22: WIPFeature/BAUTIRO-182 call data service for cluster hole in ccu mission execution
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-182-call-data-service-for-cluster-hole-in-ccu-mission-execution to develop
  * commit '82dd9b5f6f8d0815258441771db4a1f48d832868':
  removed and ignored .project
  removed old files
  add missing msg in cmakelist
  add missing interfacecs from older commit to avoid runtime error with ccu_data_services
  added reference frame
  Add feedback for FPM/RPM main tasks
  add bt start actions
* removed and ignored .project
* removed old files
* add missing msg in cmakelist
* add missing interfacecs from older commit to avoid runtime error with ccu_data_services
* pull latest updates from develop
* fixing broken build by removing service duplicates
* Pull request #20: Feature/fpm ctrl x
  Merge in BAUTIRO/bautiro_common from feature/fpm_ctrl_x to develop
  * commit '97ceb1156a19ed19dd3157603ac23247c3cbadaf':
  feat: Added service to set lift velocity profile.
  fix: Missing definitios in CMake file.
  feat: Definitions for ctrl_x modes.
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into feature/fpm_ctrl_x
* Pull request #13: Lu cleanup
  Merge in BAUTIRO/bautiro_common from lu-cleanup to develop
  * commit '51964d4dbb331604410f18fa62d560fca328b08f':
  Implement comments to last pull request #2
  Implement comments to last pull request
* added reference frame
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro_common into lu-cleanup
* Add feedback for FPM/RPM main tasks
* add bt start actions
* Pull request #19: Feature/staging develop
  Merge in BAUTIRO/bautiro_common from feature/staging_develop to develop
  * commit 'c58b409a2234beb7bfb7667c71d1c032fa496f1d':
  fix missing srvs in cmakelist
  update action goal
  add ccu bt start action
  GetTrafo general
  lu cleanup
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into feature/staging_develop
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro_common into lu-cleanup
* Implement comments to last pull request #2
* Implement comments to last pull request
* fix missing srvs in cmakelist
* Merge branch 'lu-cleanup' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into feature/staging_develop
* update action goal
* add ccu bt start action
* Pull request #18: Feature/BAUTIRO-7 evil sg82fe
  Merge in BAUTIRO/bautiro_common from feature/BAUTIRO-7_evil_sg82fe to develop
  * commit '960c3d54cc53bd9f49f8f0eff0458bcb986a2ab5':
  remove .project files
  remove .project files
  update DrillMask, rm not-used GetClusterPositions
  remove outdated  puml files
  remove unused ros-msk-package (grpc upcoming)
  AddMissionEvent with NavigationResult
  Remove unused srv and msg due to gRPC introduction.
  add rpm bt server start
  add action for moving relative to ur_base
  add 'id' in DrillHole.msg - this is a unique thing
  compatible change:  AddMissionEvent see
  add start srv
  JobSched is now only a list --> removed
  add GetMissionConfig
  compatible  change:    add geometry_msg/pose
  Add fake pose controller interfaces
  Remove default values from srv and msg definitions
* remove .project files
* update DrillMask, rm not-used GetClusterPositions
* remove outdated  puml files
* AddMissionEvent with NavigationResult
* Remove unused srv and msg due to gRPC introduction.
* add rpm bt server start
* add action for moving relative to ur_base
* feat: Added service to set lift velocity profile.
* add 'id' in DrillHole.msg - this is a unique thing
  ==>  do not need 'parent_pattern_name' with that
* compatible change:  AddMissionEvent
  see
  + drill_holes[]   @ AddMissionEvent
  + parent_pattern_name : string    @ DrillHole
  + drill_hole_state_int64 : int64     @ DrillHole
  see:  https://inside-docupedia.bosch.com/confluence/display/bautiro/AddMissionEvent
* fix: Missing definitios in CMake file.
* Merge branch 'feature/staging_develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into feature/staging_develop
* add start srv
* JobSched is now only a list --> removed
* add GetMissionConfig
* compatible  change:    add geometry_msg/pose
  @ GetClusterPosition.srv
  and  5 *.msg
* Add fake pose controller interfaces
* Remove default values from srv and msg definitions
  Update  ros interfaces
  - Target Mai-demo|sprint15|16
  - Added bautiro_ros_interfaces for:
  HCU:
  - Mission, DrillJob etc.
  FPM:
  - GetDrillHolesCluster.srv
  - GetClusterPosition.srv
  BOTH:
  - Pattern: Recursive Definition
  NOT possible in ROS2 IDL
  - Overworked in session w. Lukas
  - Introduce AddMissionEvent with Andreas
  - Not yet required for demo in
  bautiro_ros_interfaces_4_mission_4_later
  - Mission CRUD Services, cookies
  - USBDetect: added one more Field
  added "list_taskplan_on_USB/CCU" and "copy_TP2CCU"
  added Mission_does not exist to service 'set_active_mission'
  added WorkplanMetadata to CopyTaskplan.srv
  add Service: GetMarkerId @ClusterPosition
* feat: Definitions for ctrl_x modes.
* GetTrafo general
* lu cleanup
* Pull request #7: Feature add get rough position service
  Merge in BAUTIRO/bautiro_common from feature_add_get_rough_position_service to develop
  * commit '7ffe8829cf58b9f07890db5cb6a049f5195c5a00':
  add LuGetTargetPoseUrFrame
  add service for GetMarkerRoughPosition
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/bautiro_common into feature_add_get_rough_position_service
* Pull request #6: LeicaTS16 service and multi point measure
  Merge in BAUTIRO/bautiro_common from feature/leicaTS16_driver_with_multipoint to develop
  * commit '2e0ae1b471a31eabc5cd7ea10cfb9c657b982b15':
  rename services due to name convention
  LeicaTS16 service and multi point measure
* rename services due to name convention
* LeicaTS16 service and multi point measure
* add LuGetTargetPoseUrFrame
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into develop
* added position sequence service
* add service for GetMarkerRoughPosition
* Pull request #4: Feature/action to service ifc update
  Merge in BAUTIRO/bautiro_common from feature/action_to_service_ifc_update to develop
  * commit '6ee7c3657207dbbc647e43a8fcc3c7875d99b756':
  1 and 2
  added Joystick.msg|'int32 speedlevel'
  added 'Joystick.msg' to messages
  overworked HandlingUnitPose, (Set)JoystickTarget, NAME for ...
  reverted back to 'int32 response_code' for the ACTION SERVERs
  action -> service: for  MoveLiftAbsolut + ResponseCode + Progress
* fixed cmake list
* final version move action absolute
* renaming to absolute pose
* added move absolute action
* 1 and 2
* added Joystick.msg|'int32 speedlevel'
* added 'Joystick.msg' to messages
* overworked HandlingUnitPose, (Set)JoystickTarget, NAME for ...
* reverted back to 'int32 response_code' for the ACTION SERVERs
* action -> service: for  MoveLiftAbsolut + ResponseCode + Progress
* update
  Added a ResponseCode
* int32 instead of byte
* update
* update
* Added CccuMode.msg
* update
* update
* update
* updates on String Array and introduced Respone Code
* String.srv is now same Responser interface as Trigger
* Pull request #3: add String.srv
  Merge in BAUTIRO/bautiro_common from feature/add_string_srv to develop
  * commit '76a1d9c0e22fcdeac82120de851a6d6d6b44166d':
  add String.srv e.g. for choosing active taskplan
* add String.srv e.g. for choosing active taskplan
* updated CMakeLists.txt with renamed HandlingUnitPose.msg
* updated interface description
* add Lu services to CMakeLists
* fix definition of action, error during colcon build
* Merge branch 'develop' of ssh://sourcecode.socialcoding.bosch.com:7999/bautiro/bautiro_common into develop
* to Develop: MoveLiftAbsolute, SetHandlingUnitConfiguredPose, MoveHandlingUnitRelativeToWorkplane,
  Merge in BAUTIRO/bautiro_common from feature/hcu-ccu-services-4-demo to develop
  * commit 'a8b22eb37b887aabb1ebc67a89e4970015666092':
  update API for relativ HanldingUnit movement in Working Plane
  updated readme.md
  add gitignore
  updated readme.md
  minor changes
  update API for HanldingUnit and lift Actions, and Topics
  added actions
* update API for relativ HanldingUnit movement in Working Plane
* minor changes
* update API for HanldingUnit and lift Actions, and Topics
* added actions
* Contributors: Abouelainein Ahmad Waleed (PT/PJ-TOP100), Ahmad Abouelainein, Andreas Mogck, Erz Michael (CR/AAS5), Michael Erz (CR/AAS5), Mogck Andreas (CR/AAS5), Nguyen Quang Huy (CR/AAS5), Rothacker Elisa (CR/AAS4), SSL2LR, Schumacher Georg (PT/PJ-TOP100), Sinisa Slavnic (CR/APA3), Sinisa Slavnic(CR/APT5), Slavnic Sinisa (CR/APT5), bautiro@lu2, mia4si
