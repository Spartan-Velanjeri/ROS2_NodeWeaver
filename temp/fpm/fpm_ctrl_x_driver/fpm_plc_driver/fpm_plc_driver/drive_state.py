# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Definitions of the model states.
"""
from enum import Enum


class DriveState(Enum):
    """Operation states of the drive (as defined in specification)."""

    en_DEFAULT = 0
    en_WAIT = 1
    en_Not_Ready_To_Switch_On = 2
    en_Switch_On_Disabled = 3
    en_Ready_to_Switch_On = 4
    en_Switched_On = 5
    en_Operation_Mode_Pos = 6
    en_Operation_Enable = 7
    en_Quick_Stop_Active = 8
    en_Fault_Reaction_Active = 9
    en_Fault = 10
    en_Fault_DS402 = 11
