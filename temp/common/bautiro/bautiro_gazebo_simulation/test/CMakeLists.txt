find_package(ament_cmake_pytest REQUIRED)

file(GLOB TESTS *.py)

foreach(TEST ${TESTS})
    get_filename_component(TEST_NAME ${TEST} NAME_WE)
    ament_add_pytest_test(${TEST_NAME} ${TEST})
endforeach()
