cmake_minimum_required(VERSION 3.8)
project(p2p_offset_controller)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)

# further dependencies manually.

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(controller src/motion_ur_base.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs bautiro_ros_interfaces rclcpp_action)
include_directories(include)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
# Install Cpp executables
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
scripts/motion_base_link_server.py
scripts/get_cluster_server.py
scripts/tf_broadcaster.py
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
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
#   ament_cppcheck() # Many Error to solve !!!!!!!!!!!!!

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

ament_package()
