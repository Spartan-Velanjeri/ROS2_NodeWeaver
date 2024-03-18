
import streamlit as st
from time import sleep
from grpc import insecure_channel
from ccu_grpc.manual_mode_pb2_grpc import ManualModeServiceStub
from ccu_grpc.manual_mode_pb2 import (MoveLiftAbsoluteRequest, DriveArmControlMessage,
                                   DriveRpmControlMessage,
                                   SetStateMessage, SetStateMessageResponse)
from google.protobuf.empty_pb2 import Empty

# ❌  ✅   ⛔

st.set_page_config(layout="wide")
with st.sidebar:
    server_ip = st.radio('_server_ip-address_', ('localhost', '192.168.0.2', '127.0.0.1'), index=0)
    port_numb = st.text_input('_port number_', value=50052)

st.subheader('', divider=True)
lift_c0, lift_c1, lift_c2,  = st.columns(3)
lift_c0.markdown('### Lift')
b_lift = lift_c0.button("Lift - move to height")
slider_lift = lift_c1.slider('lift_target_height [m]', 0.0, 5.0, 3.7, 0.01)
lift_c2.caption('No feedback in *.proto')

st.subheader('', divider=True)
move_col1, move_col2, move_col3, move_col4 = st.columns(4)
move_col1.markdown('### HU Move')
b_move = move_col1.button('HU Move')
move_preset = move_col2.radio('_direction_hu_move_', ('LEFT', 'RIGHT', 'FORWARD', 'BACKWARD'))
sm_move = move_col3.radio('speed-multi__hu', (0.25, 0.5, 0.75, 1.0), index=1, horizontal=True)
move_col4.caption('No feedback in *.proto')

st.subheader('', divider=True)
pose_col1, pose_col2, pose_col3 = st.columns(3)
pose_col1.markdown('### HU Pose')
b_pose = pose_col1.button('HU Pose')
pose_preset = pose_col2.radio('_target_pose_', ('FINE_POSITIONING', 'DRIVING'))
pose_col2.caption('_SetState in *.proto_')
pose_col3.caption('feedback via  progress & final state')
prog_pose = pose_col3.progress(0.0)

st.subheader('', divider=True)
rpm_c0, rpm_c1, rpm_c2, rpm_c3 = st.columns(4)
rpm_c0.markdown('### RPM move')
b_rpm = rpm_c0.button("RPM drive")
fb_rpm = rpm_c1.slider('Forward-Backward__rpm',  -1.0, 1.0, 0.75, 0.05)
lr_rpm = rpm_c1.slider('Left-Right__rpm', -1.0, 1.0, -0.25, 0.05)
sm_rpm = rpm_c1.radio('speed-multi__rpm', (0.25, 0.5, 0.75, 1.0), index=1, horizontal=True)
x, y = rpm_c2.columns(2)
num_rpm = x.radio('num messages **SEND**', (1, 5, 10, 50, 100), index=2)
per_rpm = y.radio('period [ms]', (10, 40, 100, 1000), index=2)
rpm_c3.caption('No feedback in *.proto')


cc1, cc2 = st.columns(2)


cc1.subheader('', divider=True)
stop_x, stop_y = cc1.columns(2)
stop_x.markdown('### ⛔ STOP')
b_stop = stop_x.button('STOP')

cc2.subheader('', divider=True)
mc_x, mc_y = cc2.columns(2)
mc_x.markdown('### ⛔ Measure Ceiling')
b_mc = mc_x.button('Measure Ceiling')



###################################################################################################
###################################################################################################
###################################################################################################


class ManualManager():
    def __init__(self) -> None:
        self.channel = insecure_channel(f'{server_ip}:{port_numb}')
        self.stub = ManualModeServiceStub(self.channel)

    def __del__(self):
        print('Destructor aufgerufen')
        self.channel.close()

    def call(self, value):
        for resp in self.stub.SetState(SetStateMessage(target_state=value)):
            resp: SetStateMessageResponse
            which = resp.WhichOneof('set_state_message_response_type')
            obj = getattr(resp, which)
            if (which == 'progress'):
                prog_pose.progress(obj.progress)
            else:
                pose_col3.write('✅ success' if obj.success else '⛔ failed')


MM = ManualManager()

if b_rpm:
    c = rpm_c3.status('Drive Rpm Control Message', expanded=True, state="running")

    def rpm_message_generator():
        global cum
        cum = ''
        for i in range(int(num_rpm)):
            msg = DriveRpmControlMessage(forward_backward_axis=float(fb_rpm),
                                         left_right_axis=float(lr_rpm),
                                         speed_mult=float(sm_rpm))
            sleep(float(per_rpm)/1000)
            cum += f"\n{i+1}.   **F/B** {msg.forward_backward_axis:.2f}    "
            cum += f"**L/R** {msg.left_right_axis:.2f}   "
            cum += f" _speed_  {msg.speed_mult:.2f}"
            yield msg
    MM.stub.DriveRpmControlStream(rpm_message_generator())
    c.markdown(cum)
    c.update(expanded=True, state="complete")


if b_lift:
    MM.stub.MoveLiftAbsolute(MoveLiftAbsoluteRequest(target_lift_height=float(slider_lift)))


if b_pose:
    if pose_preset == 'FINE_POSITIONING':
        MM.call(SetStateMessage.TargetState.FINE_POSITIONING)
    elif pose_preset == 'DRIVING':
        MM.call(SetStateMessage.TargetState.DRIVING)
    else:
        st.write('ERROR')

if b_move:
    dirs = {'LEFT': DriveArmControlMessage.Direction.LEFT,
            'RIGHT': DriveArmControlMessage.Direction.RIGHT,
            'FORWARD': DriveArmControlMessage.Direction.FORWARD,
            'BACKWARD': DriveArmControlMessage.Direction.BACKWARD}
    msg = DriveArmControlMessage(direction=dirs[move_preset],
                                 speed_mult=float(sm_move))
    MM.stub.DriveArmControl(msg)

if b_stop:
    stop_y.markdown('⛔\n\n _**NOTSUPPORTED**_')

if b_mc:
    mc_y.markdown('⛔\n\n _**NOTSUPPORTED**_')
