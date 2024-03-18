# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import streamlit as st
from grpc import insecure_channel
from ccu_grpc.base_layer_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_pb2 import StartMissionRequest, StartMissionResponse
from google.protobuf.empty_pb2 import Empty
from ccu_grpc.base_layer_pb2 import MissionDataResponse, MissionIdRequest
from ccu_grpc.mission_list_pb2 import MissionList
from ccu_grpc.node_tree_message_pb2 import NodeTreeMessage

# ❌  ✅   ⛔

st.set_page_config(layout="wide")
with st.sidebar:
    server_ip = st.radio('_server_ip-address_', ('localhost', '192.168.0.2', '127.0.0.1'), index=0)
    port_numb = st.text_input('_port number_', value=50052)


if 'mission_list_text' not in st.session_state:
    st.session_state['mission_list_text'] = ""
if 'mission_start_text' not in st.session_state:
    st.session_state['mission_start_text'] = ""
if 'node_tree_text' not in st.session_state:
    st.session_state['node_tree_text'] = ""
if 'cluster_data_text' not in st.session_state:
    st.session_state['cluster_data_text'] = ""


c1, c2 = st.columns(2)

c1.subheader('', divider=True)
lm, con_mission_list = c1.columns(2)
lm.markdown('### List Missions')
b_list_mission = lm.button("list_mission")

c2.subheader('', divider=True)
sm, con_start_mission = c2.columns(2)
sm.markdown('### Start Mission')
id_start_mission = sm.text_input('mission_id__start_mission', "0")
b_start_mission = sm.button("Start_Mission")


c1, c2 = st.columns(2)

c1.subheader('', divider=True)
ndt, con_node_tree_data = c1.columns(2)
ndt.markdown('### Node Tree Data')
id_node_tree = ndt.text_input('mission_id__node_tree', "0")
b_node_tree_data = ndt.button("node_tree_data")

c2.subheader('', divider=True)
cd, con_cluster_data = c2.columns(2)
cd.markdown('### Cluster Data')
id_cluster_data = cd.text_input('mission_id__cluster_data', "0")
b_cluster_data = cd.button("cluster_data")

st.subheader('', divider=True)

###################################################################################################
###################################################################################################
###################################################################################################


class BaseManager():
    def __init__(self) -> None:
        self.channel = insecure_channel(f'{server_ip}:{port_numb}')
        self.stub = BaseLayerServiceStub(self.channel)

    def __del__(self):
        print('Destructor aufgerufen')
        self.channel.close()

    def start_mission(self, mission_id: int = 1) -> bool:
        resp: StartMissionResponse = self.stub.StartMission(
            StartMissionRequest(id=mission_id))
        return resp.success

    def list_mission(self):
        resp: MissionList = self.stub.GetMissionList(Empty())
        return resp.missions

    def node_tree_data(self, mission_id: int = 1) -> NodeTreeMessage:
        resp: NodeTreeMessage = self.stub.GetNodeTreeData(
            MissionIdRequest(mission_id=mission_id))
        return resp.rootNodes

    def cluster_data(self, mission_id: int = 1) -> MissionDataResponse:
        r: MissionDataResponse = self.stub.GetClusterData(
            MissionIdRequest(mission_id=mission_id))
        return r.clusters


BM = BaseManager()

# Button in der zweiten Spalte
if b_start_mission:
    s = '✅ success' if BM.start_mission(
        mission_id=int(id_start_mission)) else '⛔ Failed'
    st.session_state['mission_start_text'] = s

if b_list_mission:
    s = ""
    for m in BM.list_mission():
        s += f'<b>id:</b> {m.id} '
        s += f'<b>pre:</b> {m.predecessor_id}<br>'
        s += f'<b>name:</b> {m.name}<br>'
        s += f'<b>date</b> { m.import_date.ToDatetime()}<br><hr>'
    st.session_state['mission_list_text'] = s

if b_node_tree_data:
    r = BM.node_tree_data(mission_id=int(id_node_tree))
    st.session_state['node_tree_text'] = repr(r)

if b_cluster_data:
    r = BM.cluster_data(mission_id=int(id_cluster_data))
    st.session_state['cluster_data_text'] = repr(r)


con_start_mission.write(st.session_state['mission_start_text'])
con_mission_list._html(st.session_state['mission_list_text'], height=200, scrolling=True)
con_node_tree_data._html(st.session_state['node_tree_text'], height=220, scrolling=True)
con_cluster_data._html(st.session_state['cluster_data_text'], height=220, scrolling=True)
