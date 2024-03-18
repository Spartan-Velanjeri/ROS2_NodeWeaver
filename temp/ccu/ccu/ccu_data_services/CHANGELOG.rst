^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ccu_data_services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #16: BAUTIRO-606 BAUTIRO-693
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-693-call_fpm_skill_on_manual-BAUTIRO-606-fixes_get-obj-file_md5_etc to develop
  * commit 'eeb6fba9f9e8bc8a3b12a77ba7028c6ac69c755d':
  BAUTIRO-606 Fix: Start _mission id out of bounds
  BAUTIRO-707 introduce ManualDrillHole
  BAUTIRO-606 BAUTIRO-693
* BAUTIRO-606 Fix: Start _mission id out of bounds
* BAUTIRO-707 introduce ManualDrillHole
* BAUTIRO-606 BAUTIRO-693
  fixes for BAUTIRO-606
  - GetRoomPlanObjFile
  - md5 calculation done
  - md5-hash send to HCU (Value taken from orig. Taskplan)
  - GetNodeTree Works
  - calling Mission 0 raises  Error
  - jobs in MissionDataResponse
  changes in Data
  - remove DrillJob.0 from Le_131 for NovemberDemo
  changes in Design/Function/Behaviour
  - index >= 1 always for Mission ( end all other ..)
  -  0 indicates 'not active' OK
  - -1 indicates errors
  - ros-pkg 'ccu_ccu_interface' introduced
  - ProtoString.msg(string proto)
  - ProtoInRosSrv(string proto | string proto)
  - GenericPrimSrv(...)
  - DataService
  - publisher sends current_active_mission
  2 x per Sec to HALC
  - HALC, cgrpc  migrated to repo ccu_data_services
  Implementation for BAUTIRO-693 BAUTIRO-263
  - HALC: manual_mode -> fpm_skill  impl.and tested
  - test files for DS and HALC moved centrally to <repo>/text
* Pull request #14: Feature/BAUTIRO-606 test get roomplan.obj
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-606-test-get-roomplan.obj to develop
  * commit '43f93bf5f423ecd8a2ec8807e5d02c2eb147135b':
  BAUTIRO-606 add log statements
  BAUTIRO-573 add Le_131_novemberdemo_TaskPlan.xml
* BAUTIRO-606 add log statements
* Pull request #11: BAUTIRO-80 BAUTIRO-339 - SUPPORT FOR ROOMPLAN
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-80-cgrpc-drill-srv-one-commit to develop
  * commit '7da0410ef8077bd88c5e9375bb10fbc9b38ba269':
  BAUTIRO-80 BAUTIRO-339
* BAUTIRO-80 BAUTIRO-339
  new protofiles with msg and srv and restructured
  - BAUTIRO-339 CRoomPlanMD in ecore model CBautiro CBautiro model
  - originImportSource in all CMetaData
  - copy_dummy_data_if_no_data_there
* Pull request #10: BAUTIRO-217 Propagate marker info by data service
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-216-update-data-service-and-behavior-tree-to-send-also-marker-information-for-fine to develop
  * commit '9f4c5b970038212f57246949e50192f57ffaae25':
  BAUTIRO-217 Propagate marker info by data service
* BAUTIRO-217 Propagate marker info by data service
* fix dependency numpy >=  instead of ==
* docu update
* Pull request #9: BAUTIRO-271
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-271 to develop
  * commit '3f75fed576a449b9250b36fa1749e8bccef4e14e':
  BAUTIRO-271
* BAUTIRO-271
* Pull request #8: ProtoBuf-in-ROS/set_active_mission/docu upd.
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-234 to develop
  * commit '9baf82153bde0a4219fc736279e3f6e99018f611':
  ProtoBuf-in-ROS/set_active_mission/docu upd.
* ProtoBuf-in-ROS/set_active_mission/docu upd.
* update the start action for bt execution
* minor refactoring
* Pull request #6: Feature/hcu ccu interface definition
  Merge in BAUTIRO/ccu_data_services from feature/hcu_ccu_interface_definition to develop
  * commit 'd9bbc81480db45387b0268adaeefbbbb5a076e44':
  BAUTIRO-26  : fix import-statement in tests
  BAUTIRO-26  : fix typos
  move proto files to bautiro_common + small updates
* BAUTIRO-26  : fix import-statement in tests
* BAUTIRO-26  : fix typos
* Pull request #5: done: gRPC Base_Layer.proto offered by ccu_data_services
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-25_impl_grpc_base_layer_in_ccu_data_services to develop
  * commit '109d44f81a0fc35be47f15222bd6c2bb7e04d432':
  done: gRPC Base_Layer.proto offered by ccu_data_services
* done: gRPC Base_Layer.proto offered by ccu_data_services
  - updated readme.md, launch-files, setup.py
  --> ros2 run ccu_data_services main
  - some service respond with dummy data (and close connection)
  - grpc code-generated for new proto-files
* Pull request #4: Fix abs2rel (was forgotten before)
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-102_fix_absPose to develop
  * commit '1cbdd8433922c19083cad4dc9427f64463a0aaad':
  Fix abs2rel (was forgotten before)
* Fix abs2rel (was forgotten before)
* docu update
* Pull request #3: Feature/BAUTIRO-23 adapt ds to tp20
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-23-adapt-ds-to-tp20 to develop
  * commit 'e9a7368a8dda3e920a28fba06250b268b56c6afd':
  TODO FIXME absPose from relPose
  after Mai-Demo :
  branch for Maidemo
* TODO FIXME absPose from relPose
* after Mai-Demo :
  - re-factured and re-structured
  - PY:  CBautiro - the model (codegen)
  - PY:  CDataService - the business logic (core.py/manager.py)
  - ROS: ccu_data_service - wraps CDataService
  (- PY:  Cgrpc (codgen) repo:hcu-interfaces/*.proto
  - adapt to  TP-2.0  ( CBautiro-model / CdataService-loader )
  - all metrics data internally stored in meter
  - grpc server/client example files in ccu_data_service
  - works and cleaned up
  - tests introduced
  - CDataService with pytest
  - ccu_Data_services manually ROS services test. works!
* branch for Maidemo
  Add [pip]CBautiro and [ROS2] ccu_data_service:
  - Taskplan(MetaData): only MetaData of TaskPlan.xml file
  - Workplan          : editable 'TaskPlan' instance
  - Mission           : the 'jobs'
  - NOT YET: ProcessingData    : recorded Drill-Event stuff
  - works basically
  - TODO Python docstring
  - TODO make more robust.
  - TODO handling of processing-data (e.g. drill-events)
  - set ccu data folder via environment variable
  - introduce  filled geometry_msg/Pose..Transform
  - This is the 11st commit message:
  - all outside  interfaces  in meters
  internally it is mm
  - if 'data' folder is empty ->  init with 3 Missions
  - change from USER to HOME as base
  - add   Path().mkdir(parents=true)
  - print  "CCU-DATA-FOLDER is here :   ... "
* systemdays2022 (+ Data-Concept-docu)
* Contributors: Ahmad Abouelainein, Schumacher Georg (PT/PJ-TOP100)
