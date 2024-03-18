cmake_minimum_required(VERSION 3.8)
project(fpm_control)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

install(
  DIRECTORY config rviz launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
