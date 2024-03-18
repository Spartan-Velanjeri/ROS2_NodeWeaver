# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Generator, Any
from bautiro_developer_functions.core.fpm_skill_call import FpmSkillCallWrapper, FpmSkillCall
from ccu_grpc.manual_mode_service_pb2 import StartDrillMessage, StartDrillResponse as SdR
from ccu_util.util import drill_holes_to_drill_mask


class StartDrill(FpmSkillCallWrapper):

    def __init__(self, fsc: FpmSkillCall) -> None:
        super().__init__(fsc)

    def start_drill(self, q: StartDrillMessage, ctx) -> Generator[SdR, None, Any]:
        self.debug('MANUAL: StartDrill called by client...')
        drill_masks = drill_holes_to_drill_mask(q.drill_holes)
        return self._start_drill('drill_pattern', drill_masks)

    def _start_drill(self, skill_xml, drill_masks):
        if not self.call_skill(skill_xml=skill_xml, drill_masks=drill_masks):
            yield SdR(finished=SdR.DrillFinishedMessage(success=False))
            return

        progress = 0.0
        while not self.finished():
            progress = min(1.0, progress + 0.05)
            yield SdR(progress=progress)
            if self.wait_for_result(timeout=0.1):
                yield SdR(finished=SdR.DrillFinishedMessage(success=self.success))
                return

        yield SdR(
            finished=SdR.DrillFinishedMessage(success=self.success))
        return
