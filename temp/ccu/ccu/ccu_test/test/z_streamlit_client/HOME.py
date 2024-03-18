
import streamlit as st

st.set_page_config(layout="wide")

st.title("HALC - gRPC-client")
st.markdown("""#### base_layer

```
- start_mission (int)                        <- True/False
- get_list_of_all_missions ()                <- Result
- get_node_tree_data (int)                   <- Result
- get_cluster_data (int)                     <- Result
- get_State_Stream ()                        <- [can_sw_to_man, can_start_miss, mission_in_progress(opt)]
```

#### manual_mode

```
- get_ControlState_Stream ()                 <- [can_drive_rpm, can_raise_lift, can_lower_lift, can_move_arm]
- set_State (Drive, FINE_POSITIONING)        <- [progess, done]
- set_DriveRpmControl_Stream (LR,FB)         <-
- set_DriveArmControl (L-R-U-D, speed)       <-
- set_MoveLiftAbsolute (height)              <-

- stop()                                     <-
- get_MeasureCeiling()                       <- Result
```

#### mission_Tracking

```
- start_MissionTracking_Stream ()            <- [NodeTree, TF, MissionEvent]
```
""")
st.title(":sunglasses:")
