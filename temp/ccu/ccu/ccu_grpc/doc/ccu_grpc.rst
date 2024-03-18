###############################
Package ``ccu_grpc``
###############################

.. _protobuf: https://sourcecode.socialcoding.bosch.com/projects/BAUTIRO/repos/bautiro_common/browse/hcu_ccu_interface_definition/protos

.. attention::

    - This package is generated code (``PYTHON``)

      - Message definitions
      - Client/Server

    - according to `protobuf`_

    - Documentation of `protobuf`_ can be found at ``bautiro_common/hcu_ccu_interface_definition/doc/docs.md``

Context
  .. uml:: ccu_grpc_context.puml

Dependencies
  project dependencies
    - ccu_data_services : e.g. serialize NodeTree as protobuf
    - ccu_hcu_abstraction : usage of service-code
    - ``*.proto`` files : gRPC Service and Message Definitions

  public/ open source packages
    - grpcio
    - grpcio-tools

  .. uml:: ccu_grpc_dependencies.puml

.. automodule:: ccu_grpc
