
source /opt/ros/humble/setup.bash || source /opt/ros/galactic/setup.bash
[ -d /opt/bautiro/btr_external_ws/install ] && source /opt/bautiro/btr_external_ws/install/setup.bash

source ../../../../../../../install/setup.bash
source ../../../../../../install/setup.bash
source ../../../../../install/setup.bash
source ../../../../install/setup.bash
source ../../../install/setup.bash
source ../../install/setup.bash
source ../install/setup.bash
source ./install/setup.bash

# grpc - Base
base/cli_base_bautiro_description.py
base/cli_base_bautiro_mesh.py
base/cli_base_developer_function_get.py
base/cli_base_developer_function_call_ptu_ansteuern.py
base/cli_base_mission_data.py
base/cli_base_mission_list.py
base/cli_base_room_plan.py
base/cli_base_start_mission.py
base/cli_base_state_stream.py
base/cli_base_stream_mission_data.py

# grpc - Manual
manual/cli_manual_arm_cb.py
manual/cli_manual_control_state_stream.py
manual/cli_manual_rpm_pub.py
manual/cli_manual_set_pose.py
manual/cli_manual_start_drilling.py

# Test grpc-MissionImport
mission_import/1_valid/cli_mission_import.py
mission_import/2A_valid/cli_mission_import.py
mission_import/2B_valid_and_has_same_roomplan_as_2A/cli_mission_import.py
mission_import/3_invalid_xsd/cli_mission_import.py
mission_import/4_invalid_no_xml/cli_mission_import.py
