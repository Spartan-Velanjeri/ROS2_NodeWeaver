#############################
Package ``ccu_data_services``
#############################

Context

.. uml:: ccu_data_services_context.puml

Description
  | A ``ROS2`` package. It is basically a wrapper.
  | It's whole business logic is implemented in ``ccu_dataservice``.

  Incoming service-calls and pub-sub messaging of topics is separated into two:

**Node for** ``BAUTIRO-CORE`` :
  Service calls are delegated to ``ccu_dataservice``

  .. uml:: mds_services_core.puml

**Node for** ``HALC`` :
  | Service calls are delegated to ``ccu_dataservice``.
  | Incoming/outgoing msgs need to be de/serialized from/to ``protobuf``
  | ➡ for this package ``ccu_grpc`` is used and required. proto files are `here <https://sourcecode.socialcoding.bosch.com/projects/BAUTIRO/repos/bautiro_common/browse/hcu_ccu_interface_definition/protos>`_

  .. uml:: mds_services_halc.puml

