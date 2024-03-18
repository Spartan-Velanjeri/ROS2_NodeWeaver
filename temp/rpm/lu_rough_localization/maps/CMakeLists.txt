find_program(curl REQUIRED)

# Retrive the JFrog account username
# Default to the local machine's username
if(NOT DEFINED ENV{BTR_JFROG_USER})
    # Get machine username
    execute_process(
        COMMAND whoami
        OUTPUT_VARIABLE USERNAME
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(NOT DEFINED USERNAME)
        message(FATAL_ERROR "Failed to retrieve machine username")
    endif()

    # Get username stored in keyring
    execute_process(
        COMMAND keyring get btr-jfrog-user ${USERNAME}
        OUTPUT_VARIABLE KEYRING_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(NOT DEFINED KEYRING_USER)
        message(STATUS "Using the machine user: ${USERNAME}")
        set(BTR_JFROG_USER ${USERNAME})
    else()
        message(STATUS "Using the username stored in the keyring: ${KEYRING_USER}")
        set(BTR_JFROG_USER ${KEYRING_USER})
    endif()
else()
    set(BTR_JFROG_USER $ENV{BTR_JFROG_USER})
endif()
# Test if credentials for JFrog Artifactory are available
# if not, try with keyring

# Retrieve token using keyring
if(NOT DEFINED ENV{BTR_JFROG_TOKEN})
    execute_process(
        COMMAND keyring get btr-jfrog-token ${BTR_JFROG_USER}
        OUTPUT_VARIABLE KEYRING_PASSWORD
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT DEFINED KEYRING_PASSWORD)
        message(FATAL_ERROR "Please set the environment variable BTR_JFROG_TOKEN or provide it in the keyring")
    else()
        set(BTR_JFROG_TOKEN ${KEYRING_PASSWORD})
    endif()
else()
    set(BTR_JFROG_TOKEN $ENV{BTR_JFROG_TOKEN})
endif()

if(NOT DEFINED BTR_JFROG_USER)
    message(FATAL_ERROR "Please set the environment variable BTR_JFROG_USER")
endif()

if(NOT DEFINED BTR_JFROG_TOKEN)
    message(FATAL_ERROR "Please set the environment variable BTR_JFROG_TOKEN")
endif()

# Set JFrog URL
set(JFROG_URL https://artifactory.boschdevcloud.com/artifactory/BautiroArtifacts-local)

# Create macro to download map bags from Artifactory
# and its metadata
macro(download_map MAP_FOLDER MAP_NAME)

    # check if the local file allready exists in the share folder
    if(EXISTS ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/maps/${MAP_FOLDER})
        message(STATUS "Map ${MAP_NAME} already exists locally")
        return()
    else()
        message(STATUS "Map ${MAP_NAME} will be downloaded.")
    endif()

    # Download the maps from JFrog
    add_custom_target(download_map_${MAP_FOLDER} ALL
        COMMAND curl -u${BTR_JFROG_USER}:${BTR_JFROG_TOKEN} -L -O "${JFROG_URL}/${PROJECT_NAME}/maps/${MAP_FOLDER}/${MAP_NAME}"
        COMMENT "Downloading map ${MAP_NAME} from JFrog"
        VERBATIM
    )

    add_custom_target(download_metadata_${MAP_FOLDER} ALL
        COMMAND curl -u${BTR_JFROG_USER}:${BTR_JFROG_TOKEN} -L -O "${JFROG_URL}/${PROJECT_NAME}/maps/${MAP_FOLDER}/metadata.yaml"
        COMMENT "Downloading map metadata from JFrog"
        VERBATIM
    )

    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${MAP_NAME}
            ${CMAKE_CURRENT_BINARY_DIR}/metadata.yaml
        DESTINATION share/${PROJECT_NAME}/maps/${MAP_FOLDER})
endmacro()

file(GLOB LOCAL_DIRECTORIES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

# Download maps for all local directories
foreach(DIR ${LOCAL_DIRECTORIES})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()
