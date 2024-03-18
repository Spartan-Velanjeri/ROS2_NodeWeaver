# Tools and Modules for bautiro launch files

## Modules

### stdargs

This declares names, accessors and functions for handling the standard arguments all our launch files use. At the time of this writing, these are

#### Functions
 * `get_std_lvl1_arg_declares`: Returns a list of DeclareLaunchArgument actions for all the arguments below
 * `get_std_lvl2_arguments`: Returns a parameters dictionary for passing on launch arguments to level 2 (or lower) launch files
  
#### Launch file arguments

 * `mode`: One of `real` (for running on the robot, also the default), `gazebo_sim` (for use with gazebo simulation)
 * `hw_sample`: One of `FUS1`, `FUS2`, `FUS3` for the corresponding robot
 * `headless`: Default false. If true, don't start GUI programs
 * `use_sim_time`: If true, set this on the nodes in your launch file

#### Conditions

 * `SIM_ONLY`: Use this with actions that should _only_ be executed in simulation
 * `NOT_SIM`: Use this with actions that should _not_ be executed in simulation
 * `HEADLESS`: Checks if headless mode is set


### paths
WIP
A little support with path-names for launch files.



## Scripts


### check_sim_time_node

Runs this without any arguments and it will report whether all nodes in the system are using sim_time (as required during simulation). It will provide log data about this, and also periodically report offending nodes. If all is well, it will terminate after a while, to avoid spamming the log. 

### check_lifecycle_managers_node

This runs until lifecycle managers are active. It can be run without arguments, where it will run at least 30 seconds and until all is_active service it can find return success.
    
#### Parameters
  
  * `lifecycle_managers` -- string array of lifecycle managers to check. If given this list, it terminate immediately after all of them are active, which can be useful in launch files to use with an OnExit event handler.