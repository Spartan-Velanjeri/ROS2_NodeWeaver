^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leica_ign_gz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-12-19)
------------------
* Pull request #2: Testing Leica
  Merge in BAUTIRO/leica_simulation from prr1le/testing to develop
  * commit '630f81955c0252beba5aa246d130c206f126e4f3':
  - Check if the reference frame matces with the topic data - Check if the request has frame id and matches with the topic data
  - Check if reference frame matches with the topic - Check if the request already has a marker id and matches with the topic data
  added name changes, included features
  updates during meeting
  updated leica drawing
  resolving merge conflicts
* added name changes, included features
* the plugin accepts commands from the user and the data can be viewed in ros2 topic with frame id
* able to send user commands, default parent link inclusion in progress
* added publisher and subscriber
* added plugin for leica pose (currently displays values only in terminal)
* Contributors: Premkumar Raamkishore (PT/PJ-TOP100), prr1le
