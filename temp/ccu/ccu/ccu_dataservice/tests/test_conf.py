# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import inspect

from ccu_dataservice.conf import Conf


def test_conf():
    print('### RUNNING: '+inspect.currentframe().f_code.co_name.upper())

    candidate = Conf('__DELETE_ME__pytest__sg82fe')

    assert 23 == candidate.NUMBER_OF_NEXT_DEFAULT
    assert 101 == candidate.NUMBER_OF_NEXT_MAX
    assert '__DELETE_ME__pytest__sg82fe' == candidate.data_folder_name
    assert 'DELETE_ME' in candidate.data_folder_path
    candidate._clean()

    print('### SUCCESS: '+inspect.currentframe().f_code.co_name.upper())


def __run_all_tests():
    test_conf()


if __name__ == '__main__':
    __run_all_tests()
