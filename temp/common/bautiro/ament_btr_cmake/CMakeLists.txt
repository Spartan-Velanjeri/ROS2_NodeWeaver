cmake_minimum_required(VERSION 3.8)
project(ament_btr_cmake NONE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(ament_btr_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include("${PROJECT_NAME}-extras.cmake" NO_POLICY_SCOPE)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
endif()

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
