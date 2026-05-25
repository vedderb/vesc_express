cmake_minimum_required(VERSION 3.5)

# Default build target Logic
if(NOT DEFINED HW_NAME)
    if(DEFINED IDF_TARGET)
        if(IDF_TARGET STREQUAL "esp32s3" OR IDF_TARGET STREQUAL "esp32s3_fh4")
            set(HW_NAME "Devkit S3 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6_fh4")
            set(HW_NAME "Devkit C6 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6" OR IDF_TARGET STREQUAL "esp32c6_fh8")
            set(HW_NAME "Devkit C6 FH8")
        elseif(IDF_TARGET STREQUAL "esp32p4" OR IDF_TARGET STREQUAL "esp32p4_nrw32n16")
            set(HW_NAME "P4 NRW32N16")
        elseif(IDF_TARGET STREQUAL "esp32c3")
            set(HW_NAME "VESC Express T")
        endif()
    else()
        set(HW_NAME "VESC Express T")
    endif()

    set(HW_NAME "${HW_NAME}" CACHE STRING "VESC hardware profile to build")
endif()

# Look for hardware header files based on HW_NAME
file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
)

foreach(HW_HEADER_PATH ${HW_HEADER_PATHS})
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_HEADER_PATH}" FILE_CONTENTS)
    set(IS_HW_MATCH false)
    set(FILE_HW_TARGET "")
    set(FILE_HW_INTERNAL_FS 0)
    set(FILE_HW_IS_SLAVE 0)

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)([ \t]+(.*))?$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_3})

            # Remove surrounding quotes
            if(DEFINED DEF_VAL AND DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME" AND DEF_VAL STREQUAL HW_NAME)
                set(IS_HW_MATCH true)
            endif()

            if (IS_HW_MATCH)
                if (DEF_KEY STREQUAL "HW_TARGET")
                    set(FILE_HW_TARGET ${DEF_VAL})
                elseif(DEF_KEY STREQUAL "HW_INTERNAL_FS")
                    set(FILE_HW_INTERNAL_FS 1)
                elseif(DEF_KEY STREQUAL "HW_IS_SLAVE")
                    set(FILE_HW_IS_SLAVE 1)
                endif()
            endif()
        endif()
    endforeach()

    if(IS_HW_MATCH AND NOT FILE_HW_TARGET STREQUAL "")
        set(HW_TARGET ${FILE_HW_TARGET})
        set(HW_FILE_NAME ${HW_HEADER_PATH})
        set(HW_INTERNAL_FS ${FILE_HW_INTERNAL_FS})
        set(HW_IS_SLAVE ${FILE_HW_IS_SLAVE})
        break()
    endif()
endforeach()

if(DEFINED HW_NAME AND DEFINED HW_TARGET AND DEFINED HW_FILE_NAME)
    # HW_TARGET must include variant suffix, e.g. esp32c6_fh4.
    string(REGEX MATCH "^([a-z0-9]+)(_.+)?$" HW_TARGET_MATCH "${HW_TARGET}")
    if(HW_TARGET_MATCH)
        set(HW_IDF_TARGET ${CMAKE_MATCH_1})
    else()
        set(HW_IDF_TARGET ${HW_TARGET})
    endif()

    # Get file name without extension
    string(REGEX MATCH "([^/]+)$" HW_FILE ${HW_FILE_NAME})
    string(REPLACE ".h" "" HW_FILE ${HW_FILE})

    # IMPORTANT: Set IDF_TARGET before including project.cmake
    if(NOT DEFINED IDF_TARGET)
        SET(IDF_TARGET ${HW_IDF_TARGET})
    elseif(NOT IDF_TARGET STREQUAL HW_IDF_TARGET)
        message(WARNING "IDF_TARGET (${IDF_TARGET}) does not match HW_IDF_TARGET (${HW_IDF_TARGET}) derived from HW_TARGET (${HW_TARGET}) for ${HW_NAME}. This may cause build issues.")
    endif()
    
    # Pass through hw file paths
    add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")

    set(SDKCONFIG_DEFAULTS_FILES "")
    set(BASE_SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_IDF_TARGET}")
    set(HW_TARGET_SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_TARGET}")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
        list(APPEND SDKCONFIG_DEFAULTS_FILES "${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
        message(STATUS "Using target sdkconfig defaults ${HW_TARGET_SDKCONFIG_DEFAULTS_FILE} for ${HW_NAME}")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${BASE_SDKCONFIG_DEFAULTS_FILE}")
        list(APPEND SDKCONFIG_DEFAULTS_FILES "${BASE_SDKCONFIG_DEFAULTS_FILE}")
        message(STATUS "Using shared sdkconfig defaults ${BASE_SDKCONFIG_DEFAULTS_FILE} for ${HW_NAME}")
    else()
        message(WARNING "No sdkconfig defaults found for ${HW_NAME} (${HW_TARGET})")
    endif()

    set(HW_SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_FILE}")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_SDKCONFIG_DEFAULTS_FILE}")
        list(APPEND SDKCONFIG_DEFAULTS_FILES "${HW_SDKCONFIG_DEFAULTS_FILE}")
        message(STATUS "Using hardware sdkconfig overrides ${HW_SDKCONFIG_DEFAULTS_FILE} for ${HW_NAME}")
    endif()

    set(COMMON_LOW_RAM_DEFAULTS_FILE "sdkconfig.defaults.idf6_lowram")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_LOW_RAM_DEFAULTS_FILE}")
        list(APPEND SDKCONFIG_DEFAULTS_FILES "${COMMON_LOW_RAM_DEFAULTS_FILE}")
    endif()

    set(TARGET_LOW_RAM_DEFAULTS_FILE "sdkconfig.defaults.idf6_lowram.${HW_IDF_TARGET}")
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_LOW_RAM_DEFAULTS_FILE}")
        list(APPEND SDKCONFIG_DEFAULTS_FILES "${TARGET_LOW_RAM_DEFAULTS_FILE}")
    endif()

    if(HW_IS_SLAVE)
        set(NOWIRELESS_LOW_RAM_DEFAULTS_FILE "sdkconfig.defaults.idf6_lowram.nowireless")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${NOWIRELESS_LOW_RAM_DEFAULTS_FILE}")
            list(APPEND SDKCONFIG_DEFAULTS_FILES "${NOWIRELESS_LOW_RAM_DEFAULTS_FILE}")
        endif()
        set(VESC_ENABLE_WIFI 0)
        set(VESC_ENABLE_BLE 0)
        set(VESC_ENABLE_GNSS 0)
        set(VESC_ENABLE_DISPLAY 0)
        set(VESC_ENABLE_TOUCH 0)
        set(VESC_ENABLE_STORAGE 0)
    elseif(HW_IDF_TARGET STREQUAL "esp32p4")
        set(VESC_ENABLE_WIFI 0)
        set(VESC_ENABLE_BLE 0)
        set(VESC_ENABLE_GNSS 0)
        set(VESC_ENABLE_DISPLAY 1)
        set(VESC_ENABLE_TOUCH 1)
        set(VESC_ENABLE_STORAGE 1)
    else()
        set(VESC_ENABLE_WIFI 1)
        set(VESC_ENABLE_BLE 1)
        set(VESC_ENABLE_GNSS 1)
        set(VESC_ENABLE_DISPLAY 1)
        set(VESC_ENABLE_TOUCH 1)
        set(VESC_ENABLE_STORAGE 1)
    endif()

    set(VESC_ENABLE_WIFI ${VESC_ENABLE_WIFI} CACHE INTERNAL "VESC WiFi feature gate" FORCE)
    set(VESC_ENABLE_BLE ${VESC_ENABLE_BLE} CACHE INTERNAL "VESC BLE feature gate" FORCE)
    set(VESC_ENABLE_GNSS ${VESC_ENABLE_GNSS} CACHE INTERNAL "VESC GNSS feature gate" FORCE)
    set(VESC_ENABLE_DISPLAY ${VESC_ENABLE_DISPLAY} CACHE INTERNAL "VESC display feature gate" FORCE)
    set(VESC_ENABLE_TOUCH ${VESC_ENABLE_TOUCH} CACHE INTERNAL "VESC touch feature gate" FORCE)
    set(VESC_ENABLE_STORAGE ${VESC_ENABLE_STORAGE} CACHE INTERNAL "VESC storage feature gate" FORCE)

    set(ENV{VESC_ENABLE_WIFI} ${VESC_ENABLE_WIFI})
    set(ENV{VESC_ENABLE_BLE} ${VESC_ENABLE_BLE})
    set(ENV{VESC_ENABLE_GNSS} ${VESC_ENABLE_GNSS})
    set(ENV{VESC_ENABLE_DISPLAY} ${VESC_ENABLE_DISPLAY})
    set(ENV{VESC_ENABLE_TOUCH} ${VESC_ENABLE_TOUCH})
    set(ENV{VESC_ENABLE_STORAGE} ${VESC_ENABLE_STORAGE})

    set(SDKCONFIG_DEFAULTS ${SDKCONFIG_DEFAULTS_FILES})
    if(NOT DEFINED SDKCONFIG)
        set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
    endif()
    message(STATUS "Using sdkconfig ${SDKCONFIG}")

    if(HW_IS_SLAVE)
        set(COMPONENTS main)
        set(EXCLUDE_COMPONENTS esp_wifi wpa_supplicant esp_coex esp_phy bt esp_lcd fatfs sdmmc wear_levelling)
        set(ENV{IDF_COMPONENT_MANAGER} 0)
        message(STATUS "Using minimal IDF component graph for no-wireless slave build")
    endif()

    # Auto-generate QML files if any *.qml files exist alongside the HW config
    get_filename_component(HW_DIR ${HW_FILE_NAME} DIRECTORY)
    file(GLOB QMLUI_QML_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_DIR}/*.qml")
    
    if(QMLUI_QML_SRCS)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        set(QMLUI_OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${HW_DIR}")
        set(QMLUI_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/tools/create_qmlui_hw.py")
        
        foreach(QMLUI_QML_SRC IN LISTS QMLUI_QML_SRCS)
            # Tell CMake to re-run configuration if the QML file or generator changes
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${QMLUI_QML_SRC} ${QMLUI_GENERATOR})
            
            message(STATUS "Generating QMLUI HW from ${QMLUI_QML_SRC}")
            execute_process(
                COMMAND ${Python3_EXECUTABLE}
                    "${QMLUI_GENERATOR}"
                    "${QMLUI_QML_SRC}" "${QMLUI_OUT_DIR}"
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE QMLUI_RESULT
                OUTPUT_VARIABLE QMLUI_OUTPUT
                ERROR_VARIABLE QMLUI_ERROR
            )
            if(NOT QMLUI_RESULT EQUAL 0)
                message(FATAL_ERROR "Failed to generate QMLUI HW files:\n${QMLUI_ERROR}")
            endif()
            message(STATUS "${QMLUI_OUTPUT}")
        endforeach()
    endif()
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")
message(STATUS "VESC feature gates: WIFI=${VESC_ENABLE_WIFI}, BLE=${VESC_ENABLE_BLE}, GNSS=${VESC_ENABLE_GNSS}, DISPLAY=${VESC_ENABLE_DISPLAY}, TOUCH=${VESC_ENABLE_TOUCH}, STORAGE=${VESC_ENABLE_STORAGE}, HW_IS_SLAVE=${HW_IS_SLAVE}")

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
