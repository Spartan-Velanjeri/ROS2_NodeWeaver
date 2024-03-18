##########################################
Package ``bautiro_developer_functions``
##########################################


.. needtable:: developer_function
   :tags: developer_function
   :style: table

.. _BAUTIRO-662 - HCU controls service and dev functions (S27): https://rb-tracker.bosch.com/tracker19/browse/BAUTIRO-662

.. _BAUTIRO-1084 - Erste Durchsprache Dev-Functions: https://rb-tracker.bosch.com/tracker19/browse/BAUTIRO-1084

.. _bautiro_common/hcu_ccu_interface_definition/base_layer_service.proto: https://github.boschdevcloud.com/BAUTIRO/bautiro_common/blob/develop/hcu_ccu_interface_definition/protos/base_layer_service.proto

.. req:: HCU controls service and dev functions [dev_func]
   :id: SW_REQ_DEVFUNC_001
   :status: accepted
   :tags: developer_function

   | realization of this Requirement is planned and tracked by
   | `BAUTIRO-662 - HCU controls service and dev functions (S27)`_
   |
   | A bautiro user (non developer) should be able
     to control service and developer functions of the Bautiro.


| ``Developer Functions`` like *'turn on/of vacuum-cleaner'*
  today are triggered by using a *terminal console* and there,
  calling ``ROS commands``, ``bash-scripts`` or execute a certain program.
| Some of these *calling* must be available on HCU side



.. spec:: HCU-to-CCU grpc BASE-SERVICE extend by GetDeveloperFunctions and CallDeveloperFunction
   :id: SW_SPEC_DEVFUNC_001
   :links: SW_REQ_DEVFUNC_001
   :tags: developer_function

   | `bautiro_common/hcu_ccu_interface_definition/base_layer_service.proto`_
   | gets extended by

     .. code::

       rpc GetDeveloperFunctions(google.protobuf.Empty) returns (DeveloperFunctionList);

       rpc CallDeveloperFunction(DeveloperFunctionDefinition) returns (DeveloperFunctionCallResult);


.. test:: ccu_data_services/test/05_grpc_HALC/base/cli_base_developer_function.py verifies the CCU-side implementation
   :id: SW_TEST_DEVFUNC_001
   :links: SW_SPEC_DEVFUNC_001
   :status: implemented
   :tags: developer_function

   First Draft ist calling ``cli_base_developer_function_get.py`` logs available functions on console output.

At the Moment this looks like this:

.. code::

   -------------------------------------------------------
   TEST: cli_base_developer_function_get.py
   -------------------------------------------------------

    call  stub.GetDeveloperFunctions()

             sources | functions                     | arguments
     --------------- | ------------------------      | ---------------------
                 fpm | set_state_rpm_driving         |
                 fpm | set_state_rpm_fine_positioning|
                  hu | hu_move_relative              | ['number: x', 'number: y']
                  hu | start_drill                   | ['number: depth']
                lift | move_lift_to_height           | ['number: target_height']
                  lu | start_feinlokalisierung       |
             mission | set_active_mission            | ['number: mission_index_to_set']
             mission | unset_active_mission          |
             mission | start_ccu_bt                  | ['number: bt_xml_enum']
     ptu_staubsauger | ptu_an                        |
     ptu_staubsauger | ptu_aus                       |
     ptu_staubsauger | ptu_ansteuern                 | ['number: drehzahl', 'bool: rechts_drehend']
     ptu_staubsauger | staubsauger_an                |
     ptu_staubsauger | staubsauger_aus               |
                 rpm | manuelles_homing              |
                 rpm | next_grobpose_anfahren        |
                 rpm | grobpose_n_anfahren           | ['number: cluster_index']
                 rpm | rpm_1_sec_geradeaus_fahren    |
                 rpm | rpm_1_sec_in_richtung_fahren  | ['number: translation', 'number: rotation', 'number: speed']
     --------------- | ------------------------      | ---------------------



.. req:: Some basic function-calls must be available
   :id: SW_REQ_DEVFUNC_002
   :status: accepted
   :tags: developer_function

   | `BAUTIRO-1084 - Erste Durchsprache Dev-Functions`_

   #. Lift auf Höhe Fahren:
   #. Manual Homing
   #. Verschiedene Behavior Trees starten
   #. Grob Pose Anfahren
   #. Feinlokalisierung anstoßen
   #. Staubsauger an / aus
   #. PTU steuern


.. spec:: New package bautiro_developer_functions is introduced.
   :id: SW_SPEC_DEVFUNC_004
   :links: SW_REQ_DEVFUNC_002
   :status: proposed
   :tags: developer_function

   | To hold the code for listing/calling ``Developer Functions``
   | and to place
   | Simple "hard coded" developer functions

   .. code::

      package:     bautiro_developer_functions
      sub-package    core     # inner logic
      sub-package    py       # callable developer functions

.. spec:: Umsetzung Some basic function-calls
   :id: SW_SPEC_DEVFUNC_002
   :links: SW_REQ_DEVFUNC_002
   :status: accepted
   :tags: developer_function


   #. Lift auf Höhe Fahren:

      - Parameter   ZielPosition     0.0  -  2.46     [0.01]
      - Parameter   Geschwindigkeit  (S,M,L)
   #. Manual Homing

   #. Grob Pose Anfahren

      - | Parameter  : Cluster n
        | -->  Send_goal(Cluster_n_koord)  and  RPM-Navigation
   #. Feinlokalsierung anstoßen

      - Parameter  : keine
   #. Staubsauger

      - Param:     an / aus
   #. PTU

      - Param: Rechts_Links
      - Param: Drehzahl


.. spec:: 'Verschiedene Behavior Trees starten' and usage of 'skill_server.get_skills()'
   :id: SW_SPEC_DEVFUNC_003
   :links: SW_REQ_DEVFUNC_001, SW_REQ_DEVFUNC_002
   :tags: developer_function

   | Regarding Requirement:  ``Verschiedene Behavior Trees starten``
   |
   | Current Implementation offers Service ``skill_server.get_skills()``
     that lists available (configured) **Skills** of the **Behavior-Trees**
   |
   | CCU-DevFunc shall use the service and (forward) offer
     these Skills as part of ``DeveloperFunctionList`` towards HCU.




