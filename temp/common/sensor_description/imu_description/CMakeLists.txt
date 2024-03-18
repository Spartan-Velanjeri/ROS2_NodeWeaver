cmake_minimum_required(VERSION 3.8)
project(imu_description)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
