cmake_minimum_required(VERSION 3.8)
project(fpm_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
