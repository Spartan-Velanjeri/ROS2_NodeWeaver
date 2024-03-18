cmake_minimum_required(VERSION 3.8)
project(fpm_moveit)

find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)

find_package(xacro REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

set(UR_TYPE ur16e)
set(UR_PREFIX hu_)
set(UR_NAME "")

# =====================================================
message(STATUS "Generating SRDF for robot: ${UR_TYPE}")
set(
  OUTPUT_SRDF_FILE
    ${PROJECT_BINARY_DIR}/srdf/ur.srdf)

xacro_add_xacro_file(
  srdf/ur.srdf.xacro
  REMAP
    # Path to installation path for all assets needed to build the
    # robot, since $(find PKG) will not work for the local package
    # in the templates
    installation_path:=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
    # Prefix for the UR robot arm installed on the FPM unit
    prefix:=${UR_PREFIX}
    name:=${UR_NAME}
)

set(OUTPUT_SRDF_FILE ${XACRO_OUTPUT_FILE})

add_custom_target(
  generate_srdf_${ROBOT}
  DEPENDS
    ${OUTPUT_SRDF_FILE}
)

add_custom_target(
  ${PROJECT_NAME}_generate_srdf_${ROBOT}
  ALL
  DEPENDS ${OUTPUT_SRDF_FILE})

xacro_install(
  ${PROJECT_NAME}_generate_srdf_${ROBOT}
    ${OUTPUT_SRDF_FILE}
  DESTINATION srdf)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_common.py)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
