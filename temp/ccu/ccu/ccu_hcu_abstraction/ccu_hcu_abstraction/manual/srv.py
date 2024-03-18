# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Generator, Any
from rclpy.node import Node
from std_srvs.srv import Trigger
from google.protobuf.empty_pb2 import Empty
from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceServicer
from ccu_grpc.manual_mode_service_pb2 import (DriveArmControlMessage,
                                              SetStateMessage, SetStateMessageResponse,
                                              ControlState, StartDrillMessage, StartDrillResponse)
from bautiro_developer_functions.core.fpm_skill_call import FpmSkillCall
from bautiro_developer_functions.core.caller import Caller

from ccu_hcu_abstraction.manual.sub.drive_arm_control import DriveArmControl
from ccu_hcu_abstraction.manual.sub.drive_rpm_control_stream import DriveRpmControlStream
from ccu_hcu_abstraction.manual.sub.control_state_stream import ControlStateStream
from ccu_hcu_abstraction.manual.sub.set_state import SetState
from ccu_hcu_abstraction.manual.sub.start_drill import StartDrill


class ManualSrv(ManualModeServiceServicer):

    def __init__(self, node: Node):
        super().__init__()

        self.css = ControlStateStream(node)
        self.drcs = DriveRpmControlStream(node)
        self.dac = DriveArmControl(node)

        fsc = FpmSkillCall(node)
        self.ss = SetState(fsc)
        self.sd = StartDrill(fsc)

        self.caller = Caller(node)
        self.caller.info("Launch Service: 'ManualMode'")

    def ControlStateStream(self, q: Empty, ctx) -> Generator[ControlState, None, Any]:  # noqa N802
        return self.css.control_state_stream(ctx)

    def DriveRpmControlStream(self, request_iterator, context) -> Empty:                # noqa N802
        return self.drcs.drive_rpm_control_stream(request_iterator, context)

    def DriveArmControl(self, q: DriveArmControlMessage, context) -> Empty:             # noqa N802
        self.dac.blocking_hu_move(q)
        if context.is_active():
            return Empty()

    def StartDrill(self, q: StartDrillMessage, context) -> StartDrillResponse:          # noqa N802
        return self.sd.start_drill(q, context)

    def SetState(self, q: SetStateMessage, context) -> SetStateMessageResponse:         # noqa N802
        return self.ss.set_state(q, context)

    def Stop(self, r: Empty, context) -> Empty:                                        # noqa: N802
        self.caller.call_service_str(Trigger, '/unset_active_mission', Trigger.Request())
        if context.is_active():
            return Empty()
