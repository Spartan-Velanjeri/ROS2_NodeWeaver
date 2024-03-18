cmake_minimum_required(VERSION 3.8)
project(bautiro_ros_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  "action/*.action")

file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  "msg/*.msg")

file(GLOB_RECURSE SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  "srv/*.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
