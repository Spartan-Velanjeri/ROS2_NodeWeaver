###############################
ccu_hcu_abstraction overview
###############################


.. uml:: ccu_hcu_abstraction_context.puml

.. attention::

  - `main` is the executed main process
  - ROS communication and gRPC communication
    are in separate *threads*
  - *callback* arrows ➡ are *in-memory* function-calls
    and get handled via events and callbacks
