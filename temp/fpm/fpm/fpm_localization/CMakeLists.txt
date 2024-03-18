cmake_minimum_required(VERSION 3.8)
project(fpm_localization)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
