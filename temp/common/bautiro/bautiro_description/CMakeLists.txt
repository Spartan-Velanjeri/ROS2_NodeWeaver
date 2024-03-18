cmake_minimum_required(VERSION 3.8)
project(bautiro_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(imu_description REQUIRED)
find_package(ouster_description REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(inclinometer_description REQUIRED)
find_package(velodyne_description REQUIRED)
find_package(fpm_control REQUIRED)
find_package(urdfdom REQUIRED)
find_package(ur_description REQUIRED)

install(
  DIRECTORY launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}/
)

# ================================
# Parse and install XACRO files
# ================================
# List of robots
set(ROBOTS FUS1 FUS2)

# Input parameters
set(UR_TYPE ur16e)
set(UR_PREFIX hu_)
set(UR_PARENT handling_unit_base)
set(FORCE_MESHFILE_ABS_PATHS true)
set(USE_SIM true)
set(UR_SAFETY_LIMITS true)
set(UR_SAFETY_POS_MARGIN 0.15)
set(UR_SAFETY_K_POSITION 20)
set(UR_NAME ur)
set(UR_SCRIPT_FILENAME ros_control.urscript)
set(UR_OUTPUT_RECIPE_FILENAME rtde_output_recipe.txt)
set(UR_JOINT_LIMITS_FILE ${ur_description_DIR}/../config/${UR_TYPE}/joint_limits.yaml)
set(UR_LOAD_ROS2_CONTROL false)
set(UR_ROBOT_IP 0.0.0.0)

set(UR_DEFAULT_KINEMATICS_FILE ${ur_description_DIR}/../config/${UR_TYPE}/default_kinematics.yaml)
if(NOT EXISTS ${UR_DEFAULT_KINEMATICS_FILE})
  message(FATAL_ERROR "File not found: ${UR_DEFAULT_KINEMATICS_FILE}")
endif()

set(UR_DEFAULT_PHY_PARAM_FILE ${ur_description_DIR}/../config/${UR_TYPE}/physical_parameters.yaml)
if(NOT EXISTS ${UR_DEFAULT_PHY_PARAM_FILE})
  message(FATAL_ERROR "File not found: ${UR_DEFAULT_PHY_PARAM_FILE}")
endif()

set(UR_DEFAULT_VIS_PARAM_FILE ${ur_description_DIR}/../config/${UR_TYPE}/visual_parameters.yaml)
if(NOT EXISTS ${UR_DEFAULT_VIS_PARAM_FILE})
  message(FATAL_ERROR "File not found: ${UR_DEFAULT_VIS_PARAM_FILE}")
endif()

set(UR_SIM_CONTROLLERS_FILE ${fpm_control_DIR}/../config/ur_controller.yaml)
if(NOT EXISTS ${UR_SIM_CONTROLLERS_FILE})
  message(FATAL_ERROR "File not found: ${UR_SIM_CONTROLLERS_FILE}")
endif()

set(UR_INITIAL_POSITIONS_FILE ${})

foreach(ROBOT ${ROBOTS})
  # =====================================================
  message(STATUS "Generating URDF for robot: ${ROBOT}")
  set(
    OUTPUT_URDF_FILE
      ${PROJECT_BINARY_DIR}/robots/${ROBOT}/bautiro.urdf)
  set(
    OUTPUT_CHECK_URDF_FILE
      ${PROJECT_BINARY_DIR}/${ROBOT}_check_urdf.log)

  xacro_add_xacro_file(
    robots/${ROBOT}/bautiro.urdf.xacro
    REMAP
      # Path to installation path for all assets needed to build the
      # robot, since $(find PKG) will not work for the local package
      # in the templates
      installation_path:=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
      # Prefix for the UR robot arm installed on the FPM unit
      ur_type:=${UR_TYPE}
      prefix:=${UR_PREFIX}
      ur_parent:=${UR_PARENT}
      force_abs_paths:=${FORCE_MESHFILE_ABS_PATHS}
      sim_ignition:=${USE_SIM}
      safety_limits:=${UR_SAFETY_LIMITS}
      safety_pos_margin:=${UR_SAFETY_POS_MARGIN}
      safety_k_position:=${UR_SAFETY_K_POSITION}
      name:=${UR_NAME}
      script_filename:=${UR_SCRIPT_FILENAME}
      output_recipe_filename:=${UR_OUTPUT_RECIPE_FILENAME}
      joint_limit_params:=${UR_JOINT_LIMITS_FILE}
      kinematics_params:=${UR_DEFAULT_KINEMATICS_FILE}
      physical_params:=${UR_DEFAULT_PHY_PARAM_FILE}
      visual_params:=${UR_DEFAULT_VIS_PARAM_FILE}
      load_ROS2_control:=${UR_LOAD_ROS2_CONTROL}
      robot_ip:=${UR_ROBOT_IP}
      simulation_controllers:=${UR_SIM_CONTROLLERS_FILE}
      initial_positions_file:=${CMAKE_SOURCE_DIR}/config/FPM/FPM1/initial_positions.yaml
  )

  set(OUTPUT_URDF_FILE ${XACRO_OUTPUT_FILE})

  add_custom_target(
    generate_urdf_${ROBOT}
    DEPENDS
      ${OUTPUT_URDF_FILE}
  )

  add_custom_target(
    ${PROJECT_NAME}_generate_urdf_${ROBOT}
    ALL
    DEPENDS ${OUTPUT_URDF_FILE})

  add_custom_target(
    ${PROJECT_NAME}_check_urdf_${ROBOT}
    ALL
    COMMAND
      check_urdf ${OUTPUT_URDF_FILE} > ${OUTPUT_CHECK_URDF_FILE}
    COMMENT
      "Check URDF file: ${OUTPUT_URDF_FILE}"
    DEPENDS
      ${OUTPUT_URDF_FILE}
  )

  add_dependencies(${PROJECT_NAME}_check_urdf_${ROBOT}
    generate_urdf_${ROBOT})

  xacro_install(
    ${PROJECT_NAME}_generate_urdf_${ROBOT}
      ${OUTPUT_URDF_FILE}
    DESTINATION robots/${ROBOT})
endforeach()

if(BUILD_TESTING)
  ament_btr_lint()
endif()

ament_package()
