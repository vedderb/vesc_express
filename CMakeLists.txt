# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "6.0") 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)

file(GLOB_RECURSE targets RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
)

# Check if HW_TARGET is defined in the command line
if(DEFINED HW_TARGET)
    set(HW_NAME_FLAG ${HW_TARGET})
else()
    set(HW_NAME_FLAG "VESC Express T") # Can be removed in the future if multiple targets are supported
endif()

foreach(target ${targets})
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${target}" FILE_CONTENTS)

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)[ \t]+(.*)$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_2})

            # Remove surrounding quotes (if it's a string)
            if(DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME")
                set(HW_NAME ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_TARGET")
                set(HW_TARGET ${DEF_VAL})
            endif()
        endif()
    endforeach()

    # Set default HW_TARGET if not defined
    if(NOT DEFINED HW_TARGET)
        set(HW_TARGET "esp32c3")
    endif()

    # Check if target is specified from command line
    if (DEFINED HW_NAME_FLAG)
        if (DEFINED HW_NAME_FLAG AND HW_NAME_FLAG STREQUAL HW_NAME)
            # Target is specified in flag and matches
            set(HW_NAME ${HW_NAME_FLAG})
        else()
            message("Skipping target ${HW_NAME} on ${HW_TARGET} as it doesn't match arg value: ${HW_NAME_FLAG}")
            # Target is specified in flag but doesn't match
            unset(HW_NAME)
            unset(HW_TARGET)
        endif()
    endif()

    if(DEFINED HW_NAME AND DEFINED HW_TARGET)
        #get last part of the path
        string(REGEX MATCH "([^/]+)$" HW_FILE ${target})
        string(REGEX REPLACE ".h" "" HW_FILE ${HW_FILE})
        message("Processing target ${HW_NAME} on ${HW_TARGET} ${target}") 
        #set(ESP_IDF_TARGET HW_TARGET)
        file(COPY "sdkconfig.${HW_TARGET}" DESTINATION sdkconfig)
        #execute idf.py set-target ${HW_TARGET}
        #execute_process(COMMAND idf.py set-target ${HW_TARGET})

        add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")
    endif()
endforeach()