# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import inspect

from ccu_dataservice.validator import TPValidator, validate_with_exception
from pkg_resources import resource_filename


def test_validate():
    print('### RUNNING: '+inspect.currentframe().f_code.co_name.upper())

    xml = resource_filename('ccu_dataservice', 'mission_files_1/TaskPlan-2.1.xml')
    # pythonic way
    assert None is validate_with_exception(xml)

    v = TPValidator()
    assert v.validate(xml)
    assert '' == v.error_text

    print('### SUCCESS: '+inspect.currentframe().f_code.co_name.upper())


if __name__ == '__main__':
    test_validate()
