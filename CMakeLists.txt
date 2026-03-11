cmake_minimum_required(VERSION 3.5)

# You can set a HW here when e.g. building with the eclipse plugin
# set(HW_NAME "Wcore")
# set(HW_NAME "Devkit C6")
set(HW_NAME "WS147T-C6")
# set(HW_NAME "VESC Assist")

# Default build target Logic
if(NOT DEFINED HW_NAME)
    if(DEFINED IDF_TARGET)
        if(IDF_TARGET STREQUAL "esp32s3")
            set(HW_NAME "Devkit S3")
        elseif(IDF_TARGET STREQUAL "esp32c6")
            set(HW_NAME "Devkit C6")
        else()
            set(HW_NAME "VESC Express T")
        endif()
    else()
        set(HW_NAME "VESC Express T")
    endif()
endif()

# Look for hardware header files based on HW_NAME
file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
    components/*/main/hwconf/hw_*.h

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
                set(HW_FILE_NAME ${HW_HEADER_PATH})
                break()
            endif()
        endif()
    endforeach()
endforeach()

if(DEFINED HW_NAME AND DEFINED HW_TARGET AND DEFINED HW_FILE_NAME)
    # Get file name without extension
    string(REGEX MATCH "([^/]+)$" HW_FILE ${HW_FILE_NAME})
    string(REPLACE ".h" "" HW_FILE ${HW_FILE})

    # IMPORTANT: Set IDF_TARGET before including project.cmake
    if(NOT DEFINED IDF_TARGET)
        SET(IDF_TARGET ${HW_TARGET})
    elseif(NOT IDF_TARGET STREQUAL HW_TARGET)
        message(WARNING "IDF_TARGET (${IDF_TARGET}) does not match HW_TARGET (${HW_TARGET}) from ${HW_NAME}. This may cause build issues.")
    endif()
    
    # Scan hardware header for exclusion flags
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_FILE_NAME}" HW_HEADER_CONTENT)
    foreach(LINE IN LISTS HW_HEADER_CONTENT)
        # Check for exclusion defines (with or without comments)
        if(LINE MATCHES "^#define[ \t]+HW_EXCLUDE_LISP")
            set(HW_EXCLUDE_LISP TRUE)
            message(STATUS "Hardware requests: Exclude LispBM")
        endif()
        if(LINE MATCHES "^#define[ \t]+HW_EXCLUDE_LEGACY_I2C")
            set(HW_EXCLUDE_LEGACY_I2C TRUE)
            message(STATUS "Hardware requests: Exclude legacy I2C drivers")
        endif()
        if(LINE MATCHES "^#define[ \t]+HW_NO_DISPLAY_DRIVERS")
            set(HW_NO_DISPLAY_DRIVERS TRUE)
            message(STATUS "Hardware requests: Exclude display drivers")
        endif()
    endforeach()
    
    # Pass through hw file paths
    add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")

# Set up sdkconfig.defaults chain
# Base defaults + target-specific + component-specific (if from component)
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults.${HW_TARGET}")
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults.${HW_TARGET}")
endif()

# Check if hardware is from a component and add component's sdkconfig.defaults
if(HW_FILE_NAME MATCHES "^components/([^/]+)/")
    set(HW_COMPONENT_NAME ${CMAKE_MATCH_1})
    set(COMPONENT_SDKCONFIG "${CMAKE_CURRENT_SOURCE_DIR}/components/${HW_COMPONENT_NAME}/sdkconfig.defaults")
    if(EXISTS ${COMPONENT_SDKCONFIG})
        list(APPEND SDKCONFIG_DEFAULTS ${COMPONENT_SDKCONFIG})
        message("Adding component sdkconfig: ${COMPONENT_SDKCONFIG}")
    endif()
endif()

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 

# Include ESP-IDF project cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

include(cmake/git_rev_parse.cmake)

git_describe(GIT_COMMIT_HASH ".")

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
