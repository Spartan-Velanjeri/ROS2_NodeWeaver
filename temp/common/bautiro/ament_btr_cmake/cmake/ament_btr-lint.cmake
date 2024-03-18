# Copyright 2022-2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.

file(GLOB_RECURSE _BTR_XML_FILES FOLLOW_SYMLINKS
  "package.xml"
  "*.xacro"
  "*.sdf"
  "*.world"
  "*.launch"
  "*.urdf"
)

file(GLOB_RECURSE _BTR_CPP_FILES FOLLOW_SYMLINKS
    "*.c"
    "*.cc"
    "*.cpp"
    "*.cxx"
    "*.h"
    "*.hh"
    "*.hpp"
    "*.hxx"
)

file(GLOB_RECURSE _BTR_PY_FILES FOLLOW_SYMLINKS
  "*.py"
)

#
# Add a test to check for the license declared in the package.xml.
#
# @public
#
function(ament_btr_check_license)
  set(LICENSE_STR "<license>Robert Bosch GmbH proprietary</license>")

  set(_PACKAGE_XML "${CMAKE_SOURCE_DIR}/package.xml")

  if(NOT EXISTS ${_PACKAGE_XML})
    message(FATAL_ERROR "No package.xml found in the root of the package")
  endif()

  file(READ ${_PACKAGE_XML} _PACKAGE_XML_CONTENT)

  string(FIND "${_PACKAGE_XML_CONTENT}" "${LICENSE_STR}" MATCH_RES)

  if(${MATCH_RES} EQUAL -1)
    message(FATAL_ERROR "No license ${LICENSE_STR} found in ${_PACKAGE_XML}")
  endif()
endfunction()

#
# Add a test to check for the copyright notice on source files.
#
# @public
#
function(ament_btr_check_copyright_notice)
  # Get current year
  string(TIMESTAMP BTR_CURRENT_YEAR "%Y")
  set(COPYRIGHT_NOTICE "Copyright 2022-${BTR_CURRENT_YEAR} Robert Bosch GmbH and its subsidiaries")

  list(APPEND _BTR_SOURCE_CODE ${_BTR_PY_FILES} ${_BTR_CPP_FILES})

  foreach(FILE ${_BTR_SOURCE_CODE})
    file(READ ${FILE} _BTR_SOURCE_FILE)
    string(FIND "${_BTR_SOURCE_FILE}" "${COPYRIGHT_NOTICE}" MATCH_RES)
    if(${MATCH_RES} EQUAL -1)
      message(FATAL_ERROR "No copyright notice <${COPYRIGHT_NOTICE}> found in ${FILE}")
    endif()
  endforeach()
endfunction()

#
# Post a warning if a test folder isn't found
#
# @public
#
function(ament_btr_check_test_folder)
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/test")
    message(WARNING "No test folder found in ${CMAKE_SOURCE_DIR}")
  endif()
endfunction()

#
# Glob all the linter calls
#
# @public
#
function(ament_btr_lint)
  # Run and configure linters
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  # Lint XML files files
  if(_BTR_XML_FILES)
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint(${_BTR_XML_FILES})
  endif()

  # Check for the correct license declaration in package.xml
  ament_btr_check_license()

    # Lint Python files
  set(BTR_PYTHON_MAX_LINE_LENGTH 120)

  if(_BTR_PY_FILES)
    find_package(ament_cmake_flake8 REQUIRED)
    ament_flake8(MAX_LINE_LENGTH ${BTR_PYTHON_MAX_LINE_LENGTH})

    find_package(ament_cmake_pep257 REQUIRED)
    ament_pep257()

    # find_package(ament_cmake_pycodestyle REQUIRED)
    # ament_pycodestyle(MAX_LINE_LENGTH ${BTR_PYTHON_MAX_LINE_LENGTH})
  endif()

  # Lint C++ files
  if(_BTR_CPP_FILES)
    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify()

    find_package(ament_cmake_cpplint REQUIRED)
    ament_cpplint()

    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck()
  endif()

  # Check for copyright notice
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  # Check for test folder
  ament_btr_check_test_folder()
endfunction()
