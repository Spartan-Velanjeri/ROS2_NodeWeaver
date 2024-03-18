#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import List
from concurrent.futures import ThreadPoolExecutor

from grpc import server as create_server

from ccu_grpc.common_types_pb2 import Matrix4x4
from ccu_grpc.base_layer_service_pb2 import (MissionDataResponse, Node,
                                             FastenerTemplate, MissionIdRequest)
from ccu_grpc.base_layer_service_pb2_grpc import (BaseLayerServiceServicer,
                                                  add_BaseLayerServiceServicer_to_server)

#
# This test is for verifying / checking
# the behavior:
#   how gRPC does the serialization
#   - for Optional field Message (-> assign with None)
#   - for Optional field string, prim (-> assign with None)
#   - for (not-Optional) fields that are not assigned ( -> assign with '' or 0 - not None)


def build_node_tree():
    nt: List[Node] = []
    nt.append(Node())                                                                            # node_tree {}
    nt.append(Node(name_unique_per_node_tree='1'))                                               # node_tree {  name_unique_per_node_tree: "1"}
    nt.append(Node(name_unique_per_node_tree='2',                                                # node_tree {  name_unique_per_node_tree: "2"}
                   children=[]))                                                                 #
    nt.append(Node(name_unique_per_node_tree='3',                                                # node_tree {  name_unique_per_node_tree: "3"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  # 	            local_transform {  }
                   series='series_who_cares_series'))                                            #   	        series: "series_who_cares_series"}
    nt.append(Node(name_unique_per_node_tree='4',                                                # node_tree {  name_unique_per_node_tree: "4"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  #              local_transform {  }
                   series='series_who_cares_series',                                             #              series: "series_who_cares_series"
                   fastener_template=FastenerTemplate(),                                         #              fastener_template {  }
                   parent_node_name_unique='parent_node_name_here'))                             #              parent_node_name_unique: "parent_node_name_here"}
    nt.append(Node(name_unique_per_node_tree='5',                                                # node_tree {	name_unique_per_node_tree: "5"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  # 	            local_transform {  }
                   series='series_who_cares_series',                                             # 	            series: "series_who_cares_series"
                   fastener_template=FastenerTemplate(parent_node_name_unique='set_to_some'),    #              fastener_template {    parent_node_name_unique: "set_to_some"  }
                   parent_node_name_unique=None))                                                # }
    nt.append(Node(name_unique_per_node_tree='6',                                                # node_tree {  name_unique_per_node_tree: "6"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  #              local_transform {  }
                   series='series_who_cares_series',                                             #              series: "series_who_cares_series"
                   fastener_template=FastenerTemplate(parent_node_name_unique=None),             #              fastener_template {  }
                   parent_node_name_unique=''))                                                  #              parent_node_name_unique: ""}
    nt.append(Node(name_unique_per_node_tree='6b',                                               # node_tree {	name_unique_per_node_tree: "6b"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  #              local_transform {  }
                   series='series_who_cares_series',                                             #              series: "series_who_cares_series"
                   fastener_template=FastenerTemplate(parent_node_name_unique=''),               #              fastener_template {  }
                   parent_node_name_unique=''))                                                  #              parent_node_name_unique: ""      }
    nt.append(Node(name_unique_per_node_tree='7',                                                # node_tree {	name_unique_per_node_tree: "7"
                   children=[],                                                                  #
                   local_transform=Matrix4x4(),                                                  #              local_transform {  }
                   series='series_who_cares_series',                                             #              series: "series_who_cares_series"
                   fastener_template=None,                                                       #
                   parent_node_name_unique=None))                                                # }
    nt.append(Node(name_unique_per_node_tree='8',                                                # node_tree {  name_unique_per_node_tree: "8"
                   series='series_who_cares_series',                                             #              series: "series_who_cares_series"
                   fastener_template=None,                                                       # }
                   parent_node_name_unique=None))                                                # room_plan_md5: "00000000000000000000000000000000"
    nt.append(Node(name_unique_per_node_tree='9',
                   series=None,
                   fastener_template=None,
                   parent_node_name_unique=None))
    return nt


class BaseSrvTest(BaseLayerServiceServicer):

    def GetMissionData(self, r: MissionIdRequest, ctx):  # noqa: N802
        return MissionDataResponse(jobs=[],
                                   room_plan_md5='0' * 32,
                                   node_tree=build_node_tree())


PORT = 50052
"""
node_tree {
}
node_tree {
  name_unique_per_node_tree: "1"
}
node_tree {
  name_unique_per_node_tree: "2"
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "3"
  series: "series_who_cares_series"
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "4"
  series: "series_who_cares_series"
  fastener_template {
  }
  parent_node_name_unique: "parent_node_name_here"
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "5"
  series: "series_who_cares_series"
  fastener_template {
    parent_node_name_unique: "set_to_some"
  }
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "6"
  series: "series_who_cares_series"
  fastener_template {
  }
  parent_node_name_unique: ""
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "6b"
  series: "series_who_cares_series"
  fastener_template {
  }
  parent_node_name_unique: ""
}
node_tree {
  local_transform {
  }
  name_unique_per_node_tree: "7"
  series: "series_who_cares_series"
}
node_tree {
  name_unique_per_node_tree: "8"
  series: "series_who_cares_series"
}
room_plan_md5: "00000000000000000000000000000000"
"""

def grpc_serve():
    server = create_server(ThreadPoolExecutor(12))
    add_BaseLayerServiceServicer_to_server(BaseSrvTest(), server)
    print(f"grpc-server listening on port {PORT}")
    server.add_insecure_port(f'[::]:{PORT}')
    server.start()
    server.wait_for_termination()


def main():
    grpc_serve()


if __name__ == '__main__':
    main()
