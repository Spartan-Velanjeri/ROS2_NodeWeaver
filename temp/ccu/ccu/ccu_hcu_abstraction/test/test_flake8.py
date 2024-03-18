# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
