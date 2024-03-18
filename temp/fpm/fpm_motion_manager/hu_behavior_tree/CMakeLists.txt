cmake_minimum_required(VERSION 3.5)
project(hu_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(hu_bt_tree_nodes REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_recoveries REQUIRED)

nav2_package()


include_directories(
  include
)

set(executable_name hu_bt_navigators)
# set(sim_executable_name sim_hu_bt_navigators)

add_executable(${executable_name}
src/main.cpp
)
# add_executable(${sim_executable_name}
# src/main_sim.cpp
# )

set(library_name ${executable_name}_core)
# set(library_name_sim ${sim_executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_bt_navigator
  hu_bt_tree_nodes
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  rosidl_default_generators
  nav2_lifecycle_manager
  nav2_recoveries
  bautiro_ros_interfaces
)

add_library(${library_name} SHARED
  src/bt_navigation.cpp
  src/tasks/bt_exec.cpp
)
# add_library(${library_name_sim} SHARED
#   src/bt_navigation_sim.cpp
#   src/tasks/bt_exec_sim.cpp
# )

ament_target_dependencies(${executable_name}
  ${dependencies}
)

# ament_target_dependencies(${sim_executable_name}
#   ${dependencies}
# )

target_link_libraries(${executable_name} ${library_name})
# target_link_libraries(${sim_executable_name} ${library_name_sim})

ament_target_dependencies(${library_name}
  ${dependencies}
)
# ament_target_dependencies(${library_name_sim}
#   ${dependencies}
# )
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${library_name_sim}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS ${sim_executable_name}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name_sim})
ament_export_dependencies(${dependencies})
ament_package()
