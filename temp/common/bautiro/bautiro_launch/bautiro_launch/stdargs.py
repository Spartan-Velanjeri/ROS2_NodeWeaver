"""Provide standard arguments that we use in all our launch files."""
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument

# ARGUMENTS

LA_MODE = "mode"
LA_MODE_REAL = "real"
LA_MODE_GAZEBO = "gazebo_sim"
LA_MODE_UR = "ur_sim"
LA_MODES = [LA_MODE_REAL, LA_MODE_GAZEBO, LA_MODE_UR]

LA_ROBOT = "hw_sample"
# This specifies the _type_ of robot, not an individual robot
# For the functional samples, each robot is its own type, but
# we expect that in the future we will have more of each type.
LA_ROBOT_FUS1 = "FUS1"
LA_ROBOT_FUS2 = "FUS2"
LA_ROBOT_FUS3 = "FUS3"
LA_ROBOTS = [LA_ROBOT_FUS1, LA_ROBOT_FUS2, LA_ROBOT_FUS3]

LA_WORLD = "world"

LA_SIM_TIME = "use_sim_time"
LC_SIM_TIME = LaunchConfiguration(LA_SIM_TIME)

LA_HEADLESS = "headless"
LC_HEADLESS = LaunchConfiguration(LA_HEADLESS)


# CONDITIONS
SIM_ONLY = LaunchConfigurationEquals(LA_MODE, LA_MODE_GAZEBO)
NOT_SIM = LaunchConfigurationEquals(LA_MODE, LA_MODE_REAL)
HEADLESS = LaunchConfigurationEquals(LA_HEADLESS, "True")


# Declare actions
DLA_MODE = DeclareLaunchArgument(LA_MODE, default_value=LA_MODE_REAL, choices=LA_MODES)
DLA_ROBOT = DeclareLaunchArgument(
    LA_ROBOT,
    default_value=LA_ROBOT_FUS2,
    description="Name of the hardware sample",
    choices=LA_ROBOTS,
)
DLA_WORLD = DeclareLaunchArgument(
    LA_WORLD, default_value="boeblingen.sdf", description="Scenario to run in"
)
DLA_HEADLESS = DeclareLaunchArgument(
    LA_HEADLESS,
    default_value="False",
    description="Start without GUI",
    choices=["True", "False"],
)


# functions
def get_std_lvl1_arg_declares():
    """Return declare launch actions for level 1 args."""
    return [DLA_MODE, DLA_ROBOT, DLA_WORLD, DLA_HEADLESS]


def get_std_lvl2_arguments():
    """Return the standard arguments for level 2 launches."""
    return [
        (arg, LaunchConfiguration(arg))
        for arg in (LA_MODE, LA_ROBOT, LA_WORLD, LA_SIM_TIME, LA_HEADLESS)
    ] + [
        ("robot_name", LaunchConfiguration(LA_ROBOT))
    ]  # TODO: FIX WORKAROUND FOR robot_name
