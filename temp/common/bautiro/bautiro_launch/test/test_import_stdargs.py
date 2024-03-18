"""Test that we can import stdargs."""

# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.


def test_import():
    import bautiro_launch.stdargs as stdargs

    assert stdargs.get_std_lvl1_arg_declares() is not None
    assert stdargs.get_std_lvl2_arguments() is not None
