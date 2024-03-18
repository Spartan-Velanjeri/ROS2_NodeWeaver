cmake_minimum_required(VERSION 3.5)
project(fpm_bt_tree_nodes)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
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
find_package(nav2_behavior_tree REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lu_fine_localization REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
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
  visualization_msgs
  lu_fine_localization
  ${OTHER_DEPS}
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
${dependencies}
)

add_library(transform_nodes SHARED src/actions/transform_clients.cpp)
list(APPEND plugin_libs transform_nodes)

#add_library(configured_pose_client_node SHARED src/actions/configured_pose_client.cpp)
#list(APPEND plugin_libs configured_pose_client_node)

#add_library(move_absolute_client_node SHARED src/actions/move_absolute_client.cpp)
#list(APPEND plugin_libs move_absolute_client_node)

add_library(move_lift_absolute_client_node SHARED src/actions/move_lift_absolute_client.cpp)
list(APPEND plugin_libs move_lift_absolute_client_node)

#add_library(lu_get_marker_rough_position_node SHARED src/actions/lu_get_marker_rough_position.cpp)
#list(APPEND plugin_libs lu_get_marker_rough_position_node)
#
#add_library(ts_measure_point_node SHARED src/actions/lu_ts_measure_point.cpp)
#list(APPEND plugin_libs ts_measure_point_node)
#
#add_library(lu_triangulation_node SHARED src/actions/lu_triangulation.cpp)
#list(APPEND plugin_libs lu_triangulation_node)
#
#add_library(lu_map_leica_tf_node SHARED src/actions/lu_map_leica_tf.cpp)
#list(APPEND plugin_libs lu_map_leica_tf_node)
#
#add_library(lu_get_target_pose_ur_frame_node SHARED src/actions/lu_get_target_pose_ur_frame.cpp)
#list(APPEND plugin_libs lu_get_target_pose_ur_frame_node)
#
#add_library(lu_base_link_desired_tcp_tf_node SHARED src/actions/lu_base_link_desired_tcp_tf.cpp)
#list(APPEND plugin_libs lu_base_link_desired_tcp_tf_node)

add_library(update_poi_node SHARED src/actions/update_poi.cpp)
list(APPEND plugin_libs update_poi_node)
add_library(load_point_node SHARED src/actions/load_points_client.cpp)
list(APPEND plugin_libs load_point_node)

add_library(get_poi_node SHARED src/actions/get_poi.cpp)
list(APPEND plugin_libs get_poi_node)

add_library(move_lift_absolute_client_sim_node SHARED src/actions/move_lift_absolute_client_sim.cpp)
list(APPEND plugin_libs move_lift_absolute_client_sim_node)


#add_library(move_absolute_client_sim_node SHARED src/actions/move_absolute_client_sim.cpp)
#list(APPEND plugin_libs move_absolute_client_sim_node)

add_library(lift_node SHARED src/actions/lift_actions.cpp)
list(APPEND plugin_libs lift_node)

add_library(hu_bt_node SHARED src/actions/hu_bt_clients.cpp)
list(APPEND plugin_libs hu_bt_node)

add_library(lu_fine_node SHARED src/actions/lu_fine_clients.cpp)
list(APPEND plugin_libs lu_fine_node)

add_library(hu_mm_node SHARED src/actions/hu_motion_manager_clients.cpp)
list(APPEND plugin_libs hu_mm_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install (TARGETS
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
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_copyright)
  ament_copyright()

  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "
    -build/header_guard,
    -build/include,
    -build/include_order,
    -build/include_what_you_use,
    -build/namespaces,
    -readability/braces,
    -readability/namespace,
    -runtime/indentation_namespace,
    -whitespace/line_length,
    -whitespace/parens,
    -whitespace/blank_line,
    -whitespace/comma,
    -whitespace/comments,
    -whitespace/end_of_line,
    -whitespace/ending_newline,
    -whitespace/newline,
    -whitespace/operators
    ")
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
