# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

MIN = 'min'
MAX = 'max'
STEP = 'step_width'


def limit(value, parameter, settings: dict):
    if parameter not in settings:
        return value
    return max(min(value, settings[parameter][MAX]), settings[parameter][MIN])
