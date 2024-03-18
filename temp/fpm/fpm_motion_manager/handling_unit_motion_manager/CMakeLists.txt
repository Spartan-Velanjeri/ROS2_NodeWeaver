cmake_minimum_required(VERSION 3.8)
project(handling_unit_motion_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

#-----------FPM MOTION MANAGER ---------------------
add_library(fpm_motion_manager_target SHARED
  src/fpm_motion_manager.cpp)
target_include_directories(fpm_motion_manager_target PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_definitions(fpm_motion_manager_target
  PRIVATE "MOTION_MANAGER_CPP_BUILDING_DLL")

ament_target_dependencies(fpm_motion_manager_target
  "bautiro_ros_interfaces"
  "geometry_msgs"
  "std_msgs"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

rclcpp_components_register_node(fpm_motion_manager_target PLUGIN "fpm_motion_manager::FPMActionServer" EXECUTABLE fpm_motion_manager)
install(TARGETS
  fpm_motion_manager_target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  bautiro_ros_interfaces
  yaml_cpp_vendor
  ${OTHER_DEPS}
)

add_executable(load_points_service
  src/load_points_service.cpp
)
ament_target_dependencies(load_points_service 
	${dependencies} 
)

add_executable(load_points_client
  src/load_points_client.cpp
)
ament_target_dependencies(load_points_client 
	${dependencies} 
)



install (TARGETS
  load_points_service
  load_points_client
  DESTINATION lib/${PROJECT_NAME}
)

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
