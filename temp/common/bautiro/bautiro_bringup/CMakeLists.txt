cmake_minimum_required(VERSION 3.8)
project(bautiro_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_btr_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  ament_btr_lint()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
