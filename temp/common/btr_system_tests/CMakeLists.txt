cmake_minimum_required(VERSION 3.8)
project(btr_system_tests)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

if(BUILD_TESTING)
  ament_btr_lint()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_subdirectory(src)
endif()

ament_package()
