cmake_minimum_required(VERSION 3.5)
project(rpm_bt_tree_nodes)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_behavior_tree)
nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  std_msgs
  std_srvs
  nav2_util
  nav2_behavior_tree
  bautiro_ros_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
  ${OTHER_DEPS}
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_pose_node SHARED src/actions/nav2_pose.cpp)
list(APPEND plugin_libs nav2_pose_node)

add_library(move_p2p_async_node SHARED src/actions/move_p2p_async.cpp)
list(APPEND plugin_libs move_p2p_async_node)

add_library(switch_point_async_node SHARED src/actions/switch_point_async.cpp)
list(APPEND plugin_libs switch_point_async_node)

add_library(transform_pose_async_node SHARED src/actions/transform_target_pose_async.cpp)
list(APPEND plugin_libs transform_pose_async_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
  #load_points
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)
# install(DIRECTORY action DESTINATION share/${PROJECT_NAME})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

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

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
