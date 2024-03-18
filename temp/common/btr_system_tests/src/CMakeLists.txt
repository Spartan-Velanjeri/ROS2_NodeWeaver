file(GLOB DIRECTORIES LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach(DIRECTORY ${DIRECTORIES})
    if(IS_DIRECTORY ${DIRECTORY})
        add_subdirectory(${DIRECTORY})
    endif()
endforeach()