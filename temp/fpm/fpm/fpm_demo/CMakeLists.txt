cmake_minimum_required(VERSION 3.8)
project(fpm_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find cmake dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find common dependencies
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)


add_executable(move_group_ex src/move_group_ex.cpp)

target_include_directories(move_group_ex PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(move_group_ex PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(move_group_ex
  "rclcpp"
  "geometry_msgs"
  "moveit_ros_planning_interface"
)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/geometry_stamped_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
