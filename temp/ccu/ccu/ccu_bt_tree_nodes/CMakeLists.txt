cmake_minimum_required(VERSION 3.8)
project(ccu_bt_tree_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)# uncomment the following section in order to fill in
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

nav2_package()

# find_package(fake_ros_interfaces REQUIRED)
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
# further dependencies manually.

# Include Cpp "include" directory
include_directories(include)
# set(library_name ${PROJECT_NAME}_core)

install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  behaviortree_cpp_v3
  bautiro_ros_interfaces 
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_behavior_tree
  # fake_ros_interfaces
  ${OTHER_DEPS}
) 


# Install Cpp executables
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)
#add_library(${library_name} SHARED
#  src/behavior_tree_engine.cpp
#)

#ament_target_dependencies(${library_name}
#  ${dependencies}
#)
add_library(data_service_client_nodes SHARED src/actions/data_service_clients.cpp)
list(APPEND plugin_libs data_service_client_nodes)

add_library(rpm_client_nodes SHARED src/actions/rpm_clients.cpp)
list(APPEND plugin_libs rpm_client_nodes)

add_library(fpm_client_nodes SHARED src/actions/fpm_clients.cpp)
list(APPEND plugin_libs fpm_client_nodes)

foreach(bt_plugins ${plugin_libs})
  ament_target_dependencies(${bt_plugins} ${dependencies})
  target_compile_definitions(${bt_plugins} PRIVATE BT_PLUGIN_EXPORT)
endforeach()
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS #${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
DESTINATION lib/${PROJECT_NAME}
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
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(
  include
)

ament_export_libraries(
  #${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
