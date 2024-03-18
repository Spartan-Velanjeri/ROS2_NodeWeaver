cmake_minimum_required(VERSION 3.16)
project(rpm_actuators)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  lely_core_libraries
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


cogen_dcf(mobile_base)
cogen_dcf(mobile_base_old)

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_copyright)
  ament_copyright()

endif()


### INSTALL ###
install(DIRECTORY config_ros2_control launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
