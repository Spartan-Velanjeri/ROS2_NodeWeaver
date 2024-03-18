# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


import streamlit as st

from grpc import insecure_channel
from google.protobuf.empty_pb2 import Empty

from ccu_grpc.tracking_mode_pb2_grpc import MissionTrackingServiceStub
from ccu_grpc.tracking_mode_pb2 import DrillResultEventList, NavigationEvent, MissionTrackingMessage
from ccu_grpc.node_tree_message_pb2 import NodeTreeMessage
from ccu_grpc.robot_transform_pb2 import RobotTransform


# ❌  ✅   ⛔

st.set_page_config(layout="wide")

with st.sidebar:
    server_ip = st.radio('_server_ip-address_', ('localhost', '192.168.0.2', '127.0.0.1'), index=0)
    port_numb = st.text_input('_port number_', value=50052)

st.title('Mission Tracking not implemented on Server Side!')
st.subheader('', divider=True)
c1, c2, c3 = st.columns(3)
c1.markdown('### ⛔ Mission Tracking')
b = c1.button("View `Tracking` stream")
num = c2.radio('num messages **RECEIVE**', (1, 5, 10, 20), index=1, horizontal=True)

st.subheader('', divider=True)

###################################################################################################
###################################################################################################
###################################################################################################


class TrackingManager():
    def __init__(self) -> None:
        self.channel = insecure_channel(f'{server_ip}:{port_numb}')
        self.stub = MissionTrackingServiceStub(self.channel)

    def __del__(self):
        print('Destructor aufgerufen')
        self.channel.close()

    def start_mission_tracking_stream(self):
        return self.stub.StartMissionTrackingStream(Empty())


TM = TrackingManager()

if b:
    c = c3.status('Tracking - Messages', expanded=True, state="running")
    i = 1
    r: MissionTrackingMessage
    for r in TM.start_mission_tracking_stream():
        if r.WhichOneof() == 'node_tree':
            result: NodeTreeMessage = r.node_tree
        elif r.WhichOneof() == 'robot_tf':
            result: RobotTransform = r.robot_tf
        elif r.WhichOneof() == 'event':
            if r.event.WhichOneof() == 'navigation_event':
                result: NavigationEvent = r.event.navigation_event
            if r.event.WhichOneof() == 'drill_result_events':
                result: DrillResultEventList = r.event.drill_result_events
        c._html(result, height=200, scrolling=True)
        i += 1
        if i > int(num):
            break
    c.update(expanded=True, state="complete")
