# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Generator, Any
from bautiro_developer_functions.core.fpm_skill_call import FpmSkillCallWrapper, FpmSkillCall

from ccu_grpc.manual_mode_service_pb2 import SetStateMessage, SetStateMessageResponse as Rsp


class SetState(FpmSkillCallWrapper):

    def __init__(self, fsc: FpmSkillCall) -> None:
        super().__init__(fsc)

    def set_state(self, q: SetStateMessage, context) -> Generator[Rsp, None, Any]:

        self.debug(f'MANUAL: SetState called by client...  Target State: {q.target_state}')

        if q.target_state == SetStateMessage.TargetState.DRIVING:
            return self._set_fpm_state('bring_fpm_to_drive_state')

        if q.target_state == SetStateMessage.TargetState.FINE_POSITIONING:
            return self._set_fpm_state('bring_fpm_to_manual_drilling_state')

    def _set_fpm_state(self, skill_xml):
        if not self.call_skill(skill_xml=skill_xml):
            yield Rsp(finished=Rsp.SetStateFinishedMessage(success=False))
            return

        progress = 0.0
        while not self.finished():
            progress = min(1.0, progress + 0.05)
            yield Rsp(progress=Rsp.SetStateProgressMessage(progress=progress))

            if self.wait_for_result(timeout=0.1):
                yield Rsp(finished=Rsp.SetStateFinishedMessage(success=self.success))
                return

        yield Rsp(finished=Rsp.SetStateFinishedMessage(success=self.success))
        return
