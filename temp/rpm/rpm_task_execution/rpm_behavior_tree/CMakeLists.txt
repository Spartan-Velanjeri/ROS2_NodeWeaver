cmake_minimum_required(VERSION 3.5)
project(rpm_behavior_tree)

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
find_package(rpm_bt_tree_nodes REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_library(bautiro_ros_interfaces REQUIRED)
nav2_package()


include_directories(
  include
)

set(executable_name rpm_bt_navigators)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav2_bt_navigator
  rpm_bt_tree_nodes
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
  #src/bt_sub.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  find_package(ament_cmake_cpplint REQUIRED)
#   ament_cpplint() # Many Error to solve !!!!!!!!!!!!!

  find_package(ament_cmake_flake8 REQUIRED)
#   ament_flake8("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.flake8) # Many Error to solve !!!!!!!!!!!!!

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()

  find_package(ament_cmake_uncrustify REQUIRED)
#   ament_uncrustify() # Many Error to solve !!!!!!!!!!!!!

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
