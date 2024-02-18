cmake_minimum_required(VERSION 3.5)

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Continued at the bottom of this file (after custom target logic)

# Default build target if not provided
if(NOT DEFINED HW_NAME)
    set(HW_NAME "VESC Express T")
endif()

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
    # Set our sdkconfig based on hw target
    file(REMOVE sdkconfig)
    configure_file(sdkconfig.${HW_TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig COPYONLY)
    # Pass through hw file paths
    add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")


# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)