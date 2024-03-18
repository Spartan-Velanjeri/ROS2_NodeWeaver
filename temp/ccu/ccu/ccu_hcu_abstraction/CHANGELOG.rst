^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ccu_hcu_abstraction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #19: BAUTIRO-781 BautiroDesc & mesh -download  impl.
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-781-start-mesh-file-transfer to develop
  * commit '217c98779290aa0a7b764d9fe3656805a19582de':
  BAUTIRO-781 BautiroDesc & mesh -download  impl.
* BAUTIRO-781 BautiroDesc & mesh -download  impl.
* Pull request #18: BAUTIRO-84 Live-Transform - Skeleton ROS-to-grpc
  Merge in BAUTIRO/ccu_data_services from feature/BAUTIRO-84-live-tf-skeletor to develop
  * commit '8c4f44850b436cbdb6cae26fe0d530027d8d7c15':
  BAUTIRO-84 Live-Transform - Skeleton ROS-to-grpc
* BAUTIRO-84 Live-Transform - Skeleton ROS-to-grpc
  - upd. to depth_meter  for Start_drill (manual)
  - Live-Transform - Skeleton ROS-to-grpc
  - upd. tracking,
  - upd file transfer
  - make  Test-files executable
  - Docu update
  BAUTIRO-82
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
* Contributors: Schumacher Georg (PT/PJ-TOP100)
