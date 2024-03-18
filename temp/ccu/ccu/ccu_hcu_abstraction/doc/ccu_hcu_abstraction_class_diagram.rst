###################################
ccu_hcu_abstraction class diagram
###################################


A more detailed look

| ``gRPC`` calls (e.g. ``GetMissionList()``) forwarded towards ``ROS`` calls
| ``ROS`` events (e.g. topic ``current_active_mission`` publish) forwarded as ``gRPC``-stream

.. uml:: ccu_hcu_abstraction_class_diagram.puml

