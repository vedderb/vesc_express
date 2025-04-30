cmake_minimum_required(VERSION 3.5)

# Default build target if not provided
if(NOT DEFINED HW_NAME)
    set(HW_NAME "VESC Express T")
endif()

# Set build directory to be target-specific
set(TARGET_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/${HW_NAME}")

# Create the build directory if it doesn't exist
file(MAKE_DIRECTORY ${TARGET_BUILD_DIR})

# Look for hardware header files
file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
)

foreach(HW_HEADER_PATH ${HW_HEADER_PATHS})
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_HEADER_PATH}" FILE_CONTENTS)
    set(IS_HW_MATCH false)

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)[ \t]+(.*)$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_2})

            # Remove surrounding quotes
            if(DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME" AND DEF_VAL STREQUAL HW_NAME)
                set(IS_HW_MATCH true)
            endif()

            # We've found the hardware target (must be below HW_NAME) - exit early
            if (IS_HW_MATCH AND DEF_KEY STREQUAL "HW_TARGET")
                set(HW_TARGET ${DEF_VAL})
                message(${HW_HEADER_PATH})
                set(HW_FILE_NAME ${HW_HEADER_PATH})
                break()
            endif()
        endif()
    endforeach()
endforeach()

if(DEFINED HW_NAME AND DEFINED HW_TARGET AND DEFINED HW_FILE_NAME)
    # Get file name without extension
    string(REGEX MATCH "([^/]+)$" HW_FILE ${HW_FILE_NAME})
    string(REGEX REPLACE ".h" "" HW_FILE ${HW_FILE})

    # IMPORTANT: Set IDF_TARGET before including project.cmake
    SET(IDF_TARGET ${HW_TARGET})
    
    # Pass through hw file paths
    add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

# Check if we need to handle sdkconfig
set(SDKCONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.${HW_TARGET}")
if(EXISTS ${SDKCONFIG_PATH})
    # Set environment variables for ESP-IDF
    set(ENV{SDKCONFIG} "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig")
    
    # Copy the appropriate sdkconfig to the root directory
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig" OR NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E copy ${SDKCONFIG_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig
        )
    endif()
endif()

message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")
message("Building in directory: ${TARGET_BUILD_DIR}")

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 

# Set ESP-IDF to build in our target-specific directory
set(CMAKE_BINARY_DIR ${TARGET_BUILD_DIR})
set(CMAKE_CURRENT_BINARY_DIR ${TARGET_BUILD_DIR})

# Include ESP-IDF project cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

include(cmake/git_rev_parse.cmake)

git_describe(GIT_COMMIT_HASH ".")
# get_git_head_revision(GIT_COMMIT_REF GIT_COMMIT_HASH ".")

if(DEFINED ENV{GIT_COMMIT_HASH})
    set(GIT_COMMIT_HASH $ENV{GIT_COMMIT_HASH})
else()
    git_describe(GIT_COMMIT_HASH ".")
endif()

if(DEFINED ENV{GIT_BRANCH_NAME})
    set(GIT_BRANCH_NAME $ENV{GIT_BRANCH_NAME})
else()
    git_rev_parse(GIT_BRANCH_NAME "." "--abbrev-ref" "HEAD")
endif()

idf_build_set_property(COMPILE_DEFINITIONS "GIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "GIT_BRANCH_NAME=\"${GIT_BRANCH_NAME}\"" APPEND)

if(DEFINED ENV{USER_GIT_COMMIT_HASH})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_COMMIT_HASH=\"$ENV{USER_GIT_COMMIT_HASH}\"" APPEND)
endif()
if(DEFINED ENV{USER_GIT_BRANCH_NAME})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_BRANCH_NAME=\"$ENV{USER_GIT_BRANCH_NAME}\"" APPEND)
endif()

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
