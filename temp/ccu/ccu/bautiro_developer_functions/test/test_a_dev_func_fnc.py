# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from bautiro_developer_functions.core.developer_functions import DeveloperFunctions


class DeveloperFunctionsTest(DeveloperFunctions):

    def get_skill_records(self):
        return []


def test_1():
    candidate = DeveloperFunctionsTest(node=None)
    candidate._build_records_cache()


if __name__ == '__main__':
    test_1()
