cmake_minimum_required(VERSION 3.8)
project(lu_fine_localization)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LuFineLocalization.srv"
  
  DEPENDENCIES
  geometry_msgs
  bautiro_ros_interfaces
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/fine_localization_service.py
  ${PROJECT_NAME}/tf2_reflex_marker_broadcaster.py
  ${PROJECT_NAME}/fine_loc_coordination.py
  ${PROJECT_NAME}/fine_loc_coordination_fus1.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.flake8)
endif()

ament_package()
