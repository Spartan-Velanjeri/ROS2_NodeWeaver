cmake_minimum_required(VERSION 3.8)
project(fpm_ceiling_distance)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bautiro_ros_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ceiling_distance_node
  src/ceiling_distance_node.cpp
  src/ceiling_distance_node_lifecycle.cpp
)

ament_target_dependencies(ceiling_distance_node
  rclcpp
  geometry_msgs
  sensor_msgs
  bautiro_ros_interfaces
  rclcpp_lifecycle
  lifecycle_msgs
)

target_include_directories(ceiling_distance_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

add_library(ceiling_distance_filter SHARED
  src/point_filters/box_filter.cpp
  src/point_filters/moving_average_filter.cpp
  src/approximation/linear_regression.cpp
  src/ceiling_distance_filter/ceiling_distance_filter.cpp
)

ament_target_dependencies(ceiling_distance_filter
  rclcpp
  geometry_msgs
  sensor_msgs
  eigen3_cmake_module
)

target_include_directories(ceiling_distance_filter PUBLIC "${Eigen3_INCLUDE_DIRS}")

target_link_libraries(ceiling_distance_node ceiling_distance_filter)

install(
  TARGETS ceiling_distance_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ceiling_distance_node
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_btr_lint()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(test_utils SHARED
              test/ut/test_utils.cpp)
  ament_target_dependencies(test_utils
                            geometry_msgs)

  ament_add_gmock(${PROJECT_NAME}_test
    test/ut/box_filter_test.cpp
    test/ut/moving_average_filter_test.cpp
    test/ut/ceiling_distance_filter_test.cpp
    test/ut/linear_regression_test.cpp
    test/ut/main.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
  )
  target_link_libraries(
    ${PROJECT_NAME}_test ceiling_distance_filter
    test_utils
  )
endif()

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
