cmake_minimum_required(VERSION 3.8)
project(fpm_handling_unit_script_switching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_controllers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )
include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  bautiro_ros_interfaces
  std_srvs
  rclcpp_components
  rcutils
  ur_dashboard_msgs
  ur_robot_driver
  ur_controllers
  ur_msgs
  sensor_msgs
  moveit_ros_visualization
  moveit_ros_planning_interface
  moveit_ros_warehouse
  rviz2
  tf2
  tf2_ros
  yaml_cpp_vendor
  rosidl_default_generators
  action_msgs
  builtin_interfaces
  ${OTHER_DEPS}
)  
set(srv_files
  srv/PointsLoad.srv
)
#rosidl_generate_interfaces(${PROJECT_NAME}
  #${srv_files}
  #DEPENDENCIES
  #  action_msgs
  #  builtin_interfaces
#)
add_executable(bt_moveit2
  src/bt_moveit2.cpp
)
ament_target_dependencies(bt_moveit2
	${dependencies} 
)

add_executable(pose_transform
  scripts/pose_transform.cpp
)
ament_target_dependencies(pose_transform
	${dependencies} 
)

add_executable(current_state
  scripts/current_state.cpp
)
ament_target_dependencies(current_state
	${dependencies} 
)

#add_executable(load_points
 # src/load_points.cpp
#)
#ament_target_dependencies(load_points
#	${dependencies} 
#)
install (TARGETS
  bt_moveit2
  pose_transform
  current_state
  #load_points
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bt_xml 
  DESTINATION share/${PROJECT_NAME}
)


# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
if( ZMQ_FOUND )
    message(STATUS "ZeroMQ found.")
    add_definitions( -DZMQ_FOUND )
else()
    message(WARNING "ZeroMQ NOT found. The execution of the BT will not be visible with Groot.")
endif()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
