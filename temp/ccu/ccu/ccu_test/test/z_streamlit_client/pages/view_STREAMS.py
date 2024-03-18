# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


import streamlit as st

from grpc import insecure_channel

from google.protobuf.empty_pb2 import Empty

from ccu_grpc.base_layer_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_pb2 import BaseLayerState

from ccu_grpc.manual_mode_pb2_grpc import ManualModeServiceStub
from ccu_grpc.manual_mode_pb2 import ControlState

# ❌  ✅   ⛔

st.set_page_config(layout="wide")

with st.sidebar:
    server_ip = st.radio('_server_ip-address_', ('localhost', '192.168.0.2', '127.0.0.1'), index=0)
    port_numb = st.text_input('_port number_', value=50052)

col1, col2, col3 = st.columns(3)


num = col1.radio('num messages **RECEIVE**', (1, 5, 10, 20), index=1, horizontal=True)
col2.write('###### Legend: \n ✅ = True (success)  \n ❌ = False (fail) ')
c1, c2 = st.columns(2)

c1.subheader('', divider=True)
base1, base2 = c1.columns(2)
base1.markdown('### Base Layer State')
b_base = base1.button("view `base_state` stream")
base1.markdown("""
               ```
               🚀 can_start_mission:
               ✋ can_switch_to_manual_mode:
               🏁 mission_in_progress: `
               ```
               """)

c2.subheader('', divider=True)
ctrl1, ctrl2 = c2.columns(2)
ctrl1.markdown('### Manual **Control State**')
b_ctrl = ctrl1.button('view `control_state` stream')
ctrl1.markdown("""
               ```
               🚙 can_drive_rpm:
               🦾 can_move_arm:
               👆 can_lower_lift:
               👇 can_raise_lift:
               ```
               """)

st.subheader('', divider=True)

###################################################################################################
###################################################################################################
###################################################################################################


class TestManager():
    def __init__(self) -> None:
        self.channel = insecure_channel(f'{server_ip}:{port_numb}')
        self.b_stub = BaseLayerServiceStub(self.channel)
        self.m_stub = ManualModeServiceStub(self.channel)

    def __del__(self):
        print('Destructor aufgerufen')
        self.channel.close()


TM = TestManager()


def pic(input: bool) -> str:
    return '✅' if input else '❌'


if b_base:
    c = base2.status('Base State - Messages', expanded=True, state="running")
    i = 1
    r: BaseLayerState
    for r in TM.b_stub.BaseLayerStateStream(Empty()):
        c.markdown(f"{i}. 🚀**{pic(r.can_start_mission)}**  . . . " +
                   f' ✋**{pic(r.can_switch_to_manual_mode)}**  . . . ' +
                   f' 🏁:       **{r.mission_in_progress}**\n')
        i += 1
        if i > int(num):
            break
    c.update(expanded=True, state="complete")

if b_ctrl:
    c = ctrl2.status('Control-State Messages', expanded=True, state="running")
    i = 1
    s: ControlState
    for s in TM.m_stub.ControlStateStream(Empty()):
        c.markdown(f'{i}. 🚙{pic(s.can_drive_rpm)} . . . ' + f' 🦾{pic(s.can_move_arm)} . . . '
                   + f' 👆{pic(s.can_lower_lift)} . . ' + f' 👇{pic(s.can_raise_lift)}\n')
        i += 1
        if i > int(num):
            break
    c.update(expanded=True, state="complete")
