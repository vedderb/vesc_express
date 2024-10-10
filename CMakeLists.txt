# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "6.0") 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

if((DEFINED ENV{HW_SRC}) OR (DEFINED ENV{HW_HEADER}))
    if(NOT DEFINED ENV{HW_SRC})
        message(FATAL_ERROR "HW_SRC not defined while HW_HEADER is set. You must either set both or none.")
    endif()
    if(NOT DEFINED ENV{HW_HEADER})
        message(FATAL_ERROR "HW_HEADER not defined while HW_SRC is set. You must either set both or none.")
    endif()
    
    idf_build_set_property(COMPILE_OPTIONS "-DHW_SOURCE=\"$ENV{HW_SRC}\"" APPEND)
    idf_build_set_property(COMPILE_OPTIONS "-DHW_HEADER=\"$ENV{HW_HEADER}\"" APPEND)
endif()

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
