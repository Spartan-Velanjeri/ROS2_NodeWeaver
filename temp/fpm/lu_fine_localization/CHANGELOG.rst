^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lu_fine_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #5: Hotfix/BAUTIRO-482 invest jenkins green again
  Merge in BAUTIRO/lu_fine_localization from hotfix/BAUTIRO-482-invest-jenkins-green-again to develop
  * commit 'db52966e068e89bf345e2eb220fc440fa06b379b':
  Update Jenkinsfile
  Add build status badge
  Move shebang to the first line of the file
  Disable testing for now
  Fix typo
  Fix folder name
  Fix typo
  Use bautiro_common in develop
  Fix repos filename
  Add license
* Update Jenkinsfile
* Add build status badge
* Move shebang to the first line of the file
* Disable testing for now
* Fix typo
* Fix folder name
* Fix typo
* Use bautiro_common in develop
* Fix repos filename
* Add license
* ERZ
* commited by ngq3fe: changes on sh construction site
* rigid_body as separate file
* Rename lu_fein... in lu_fine... bugfix
* Rename lu_fein... in lu_fine...
* Pull request #2: Feature/Leica dust position
  Merge in BAUTIRO/lu_fein_localization from feature/Leica_dust_position to develop, test with accuracymeasurement_finepos_with_robotarm was successful
  * commit 'ffc3b0a68eefe5806b1a597b9422325a1c682263':
  adaption to develop
  correction of Leica dust protection position to PositionStamped()
  wip move Leica to dust protection position (look downwards), not tested
* correction of sleeping time
* adaption to develop
* correction of Leica dust protection position to PositionStamped()
* roe2rng: wait 6s until ts16 has free view
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/lu_fein_localization into develop
* roe2rng: robot arm calibration data with robot arm in front position (20.06.2023)
* wip move Leica to dust protection position (look downwards), not tested
* README.md online editiert mit Bitbucket
* aktueller stand fuer Huy
* add calibration from ts16 to arm on front side
* add coordination for fus1 demo
* fein_loc_coordination_fus1.py
* Working during eval measurements
* gt measurements 1TS
* single ts
* Small changes for Video
* adaption for evaluation measurements
* Marker-Konstellationen fuer Eval
* bug fix
* eval next version
* Letzte Aenderungen rueckgaengig
* Bugfix transform_services.py
* fein_loc_coordination_wo_hw
* prism trafos for base_link_fein
* Vorbereitung eval
* wip, not finished, not working, preparation for validation measurements
* fein_loc_coordination_wo_robotarm
* Switch from TS60 to TS16
* updated index.rst with basic content
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/lu_fein_localization into develop
* launch.json
* Warking handling unit action call
* add doc folder, jenkin file, add move robot by calling action from FPM
* describe dependencies and README
* demo
* bugfixes
* lab131 15.11
* demo control sequecen with quick implemented action call (sleep)
* minor changes for lab131
* merge conflicts
* bug fix for TF problem in fine localization
* also publish rough position of robot base
* increase wait time in tf buffer request a bit
* fix while testing
* Merge branch 'develop' of https://sourcecode.socialcoding.bosch.com/scm/bautiro/lu_fein_localization into develop
* add target to marker list, add tf for leica_fein and ur16_fein
* replace .gitignore file
* Add new Festpunkte for Le131
* Fix subscriber topics, add Leica pose publisher
* Neue Festpunkte fuer Le131
* Konsistente Dateinamen
* Konsistente Dateinamen
* add .gitignore
* changes after first test in LE131
* small bug fix on wägele
* add definition of LuControlTs60
* Pull request #1: Master
  Merge in BAUTIRO/lu_fein_localization from master to develop
  * commit '3cfb016775cb615bb2ef8a4333337bc9543ff6fd':
  change bautiro_srvs to bautiro_ros_interfaces
  add service call for TS60
* change bautiro_srvs to bautiro_ros_interfaces
* add service call for TS60
* Initial Commit; move triangulation service, static transform publisher
* Contributors: =, Erz Michael (CR/AAS5), Marcusso Manhaes Musa Morena (CR/AAS3), Michael Erz (CR/AAS5), Musa Morena Marcusso Manhaes, Nguyen Quang Huy (CR/AAS5), Nguyen, Quang Huy (CR/AAS5), Rothacker Elisa (CR/AAS4), Schumacher Georg (PT/PJ-TOP100), bautiro, bautiro from rpm, bautiro@lu2
