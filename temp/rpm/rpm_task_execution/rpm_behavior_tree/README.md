## Behaviour Server:

The role of the Behaviour server is to extract information from requests to pass to the behavior tree / blackboard, populate feedback and responses. The behaviour tree XML will define the actual logic used.

For concept on behavior server and steps to write your own behavior server plugin, please refer [Behavior server](https://navigation.ros.org/plugin_tutorials/docs/writing_new_navigator_plugin.html#overview).

Requirements to run this behavior server:

- ROS2
- [nav2](https://github.com/ros-planning/navigation2.git) (with dependencies)

## Bt_server Implementation:

## bt_navigation.cpp:

The tree nodes from bt_tree_nodes package are loaded as plugins. It manages the execution of behaviors,in this package there are two behavior plugins namely first_navigator_ and second_navigator_.

## Navigator.hpp:

Navigate Muxer class controls the state of the BT navigator by allowing only a single plugin to be processed at a time.

Navigator interface acts as a base class for all BT-based action's plugins. It contains several functions which are called by the bt_navigation.cpp.

One important function is on_configure(): It creates the behavior tree action server and sets some blackboard variables.

## bt_exec.cpp:

It is the main source file for first_navigator_ behavior. The behaviour genrally contains several virtual methods and the state transition is managed by lifecycle manager. The several virtual methods are:

- configure() : Method is called when Navigator.hpp is in on_configure state, It implement operations which are neccessary before navigator goes to active state, like getting parameters and setting up the blackboard.

- activate() : Method is called when navigator is in on_activate state, It creates clents and subscriptions, if needed.

- goalReceived(): Method is called when new goal is received by action server. If the goal is accepted, It will load the neccessary parameters from request to the blackboard variables.

- onLoop(): Method is called periodically while the behavior tree is looping to check the statuses and publish action feedback statuses to client.

- goalCompleted(): Method is called when a goal is completed to populate the action result.

- getDefaultBTFilepath(): Method is called to retrieve the default BT filepath.

NOTE: The virtual methods in Navigator.hpp calls user defined functions for any additional user-specific needs. This means on_configure() method in navigation.hpp calls the configure() method in bt_exec.cpp.

## bt_sub.cpp:

It is the main source file for second_navigator_  behavior. The implementation is similar to bt_exec.cpp.

## Implementation:

Launch the behaviour server and Lifecycle manager:

             ros2 launch bt_server demo_server.launch.py

Send the goal to the first_navigator_ behavior:

             ros2 action send_goal /add_int bt_tree_nodes/action/AddInt "{totalint: 10}"

The action server "/add_int" runs until the variable is added up to 10 and returns the Result message as "succeeded".

Send the goal to the second_navigator_ behavior:

            ros2 action send_goal /subtract_int bt_tree_nodes/action/SubtractInt "{start: true}"

The action server "/subtract_int" subtracts 1 from variable value, until the variable value becomes 0 and returns the result message as "succeeded".

