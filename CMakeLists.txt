cmake_minimum_required(VERSION 3.22)

# You can set a HW here when e.g. building with the eclipse plugin
# set(HW_NAME "Wcore")

# Default build target Logic
if(NOT DEFINED HW_NAME OR "${HW_NAME}" STREQUAL "")
    if(DEFINED IDF_TARGET)
        if(IDF_TARGET STREQUAL "esp32")
            set(HW_NAME "Devkit ESP32 4MB")
        elseif(IDF_TARGET STREQUAL "esp32s3")
            set(HW_NAME "Devkit S3 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6")
            set(HW_NAME "Devkit C6 FH4")
        elseif(IDF_TARGET STREQUAL "esp32p4")
            set(HW_NAME "P4 NRW32N16")
        elseif(IDF_TARGET STREQUAL "esp32c3")
            set(HW_NAME "VESC Express T")
        elseif(IDF_TARGET STREQUAL "esp32s3_fh4")
            set(HW_NAME "Devkit S3 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6_fh4")
            set(HW_NAME "Devkit C6 FH4")
        elseif(IDF_TARGET STREQUAL "esp32p4_nrw32n16")
            set(HW_NAME "P4 NRW32N16")
        endif()
    endif()

    if(NOT DEFINED HW_NAME OR "${HW_NAME}" STREQUAL "")
        set(HW_NAME "VESC Express T")
    endif()
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
    set(FILE_HW_NO_WIFI 0)
    set(FILE_HW_NO_BLE 0)

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)([ \t]+(.*))?$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_3})

            # Remove surrounding quotes
            if(DEFINED DEF_VAL AND DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME" AND DEF_VAL STREQUAL "${HW_NAME}")
                set(IS_HW_MATCH true)
            endif()

            if (DEF_KEY STREQUAL "HW_TARGET")
                set(FILE_HW_TARGET ${DEF_VAL})
            elseif(DEF_KEY STREQUAL "HW_INTERNAL_FS")
                set(FILE_HW_INTERNAL_FS 1)
            elseif(DEF_KEY STREQUAL "HW_NO_WIFI")
                set(FILE_HW_NO_WIFI 1)
            elseif(DEF_KEY STREQUAL "HW_NO_BLE")
                set(FILE_HW_NO_BLE 1)
            endif()
        endif()
    endforeach()

    if(IS_HW_MATCH AND NOT "${FILE_HW_TARGET}" STREQUAL "")
        set(HW_TARGET ${FILE_HW_TARGET})
        set(HW_FILE_NAME ${HW_HEADER_PATH})
        set(HW_INTERNAL_FS ${FILE_HW_INTERNAL_FS})
        set(HW_NO_WIFI ${FILE_HW_NO_WIFI})
        set(HW_NO_BLE ${FILE_HW_NO_BLE})
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
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_FILE_NAME}")

    set(SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_IDF_TARGET}")
    set(HW_TARGET_SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_TARGET}")

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
        set(SDKCONFIG_DEFAULTS_FILE "${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
    else()
        message(STATUS "Using shared sdkconfig defaults ${SDKCONFIG_DEFAULTS_FILE} for ${HW_NAME}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${SDKCONFIG_DEFAULTS_FILE}")
    set(SDKCONFIG_DEFAULTS ${SDKCONFIG_DEFAULTS_FILE})

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

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 

if(SDKCONFIG)
    get_filename_component(sdkconfig_path "${SDKCONFIG}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
else()
    set(sdkconfig_path "${CMAKE_SOURCE_DIR}/sdkconfig")
endif()
set(sdkconfig_board_key "${HW_NAME}\n${HW_TARGET}\n")
if(HW_NO_WIFI OR HW_NO_BLE)
    string(APPEND sdkconfig_board_key "HW_NO_WIFI=${HW_NO_WIFI}\nHW_NO_BLE=${HW_NO_BLE}\n")
endif()
set(sdkconfig_owner_file "${sdkconfig_path}.board")
set(sdkconfig_previous_board "")
if(EXISTS "${sdkconfig_owner_file}")
    file(READ "${sdkconfig_owner_file}" sdkconfig_previous_board)
endif()
if(NOT "${sdkconfig_previous_board}" STREQUAL "${sdkconfig_board_key}" AND EXISTS "${sdkconfig_path}")
    message(STATUS "Hardware changed or sdkconfig ownership unknown; regenerating ${sdkconfig_path} from ${SDKCONFIG_DEFAULTS}")
    file(REMOVE "${sdkconfig_path}")
endif()

set(ENV{HW_NO_WIFI} "${HW_NO_WIFI}")
set(radio_disabled_symbols "")
if(HW_NO_WIFI)
    list(APPEND radio_disabled_symbols ESP_WIFI_ENABLED ESP_HOST_WIFI_ENABLED ESP_WIFI_REMOTE_ENABLED)
    if(HW_IDF_TARGET STREQUAL "esp32p4")
        list(APPEND radio_disabled_symbols ESP_HOSTED_HOST_FEAT_WIFI)
    endif()
endif()
if(HW_NO_BLE)
    list(APPEND radio_disabled_symbols BT_ENABLED)
    if(HW_IDF_TARGET STREQUAL "esp32p4")
        list(APPEND radio_disabled_symbols ESP_HOSTED_HOST_FEAT_BT)
    endif()
endif()
if(HW_NO_WIFI AND HW_NO_BLE AND HW_IDF_TARGET STREQUAL "esp32p4")
    list(APPEND radio_disabled_symbols ESP_HOSTED)
endif()

if(radio_disabled_symbols)
    set(radio_defaults "")
    foreach(symbol IN LISTS radio_disabled_symbols)
        string(APPEND radio_defaults "CONFIG_${symbol}=n\n")
    endforeach()
    set(radio_defaults_path "${CMAKE_BINARY_DIR}/sdkconfig.hw_radios")
    file(CONFIGURE OUTPUT "${radio_defaults_path}" CONTENT "${radio_defaults}" @ONLY)
    list(APPEND SDKCONFIG_DEFAULTS "${radio_defaults_path}")

    if(EXISTS "${sdkconfig_path}")
        file(READ "${sdkconfig_path}" radio_sdkconfig)
        foreach(symbol IN LISTS radio_disabled_symbols)
            string(REGEX REPLACE "(^|\n)(CONFIG_${symbol}=[^\n]*|# CONFIG_${symbol} is not set)(\n|$)" "\n" radio_sdkconfig "${radio_sdkconfig}")
        endforeach()
        string(REGEX REPLACE "\n+$" "" radio_sdkconfig "${radio_sdkconfig}")
        file(WRITE "${sdkconfig_path}" "${radio_sdkconfig}\n${radio_defaults}")
    endif()
    message(STATUS "Hardware radio exclusions: Wi-Fi=${HW_NO_WIFI}, BLE=${HW_NO_BLE}")
endif()

# Include ESP-IDF project cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(VESC_HW_FILE ${HW_FILE})

# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

idf_build_get_property(config_env_path CONFIG_ENV_PATH)
file(READ "${config_env_path}" config_env)
string(JSON config_env SET "${config_env}" HW_NO_WIFI "\"${HW_NO_WIFI}\"")
file(CONFIGURE OUTPUT "${config_env_path}" CONTENT "${config_env}\n" @ONLY)

if(HW_NO_WIFI)
    if(CONFIG_SOC_WIFI_SUPPORTED)
        idf_component_get_property(wifi_lib esp_wifi COMPONENT_LIB)
        get_target_property(wifi_sources ${wifi_lib} SOURCES)
        list(FILTER wifi_sources EXCLUDE REGEX "/remote/")
        set_property(TARGET ${wifi_lib} PROPERTY SOURCES "${wifi_sources}")
    endif()
    if(CONFIG_ESP_WIFI_ENABLED OR CONFIG_ESP_HOST_WIFI_ENABLED OR CONFIG_ESP_WIFI_REMOTE_ENABLED)
        message(FATAL_ERROR "HW_NO_WIFI requires all Wi-Fi drivers to be disabled")
    endif()
endif()
if(HW_NO_BLE AND CONFIG_BT_ENABLED)
    message(FATAL_ERROR "HW_NO_BLE requires CONFIG_BT_ENABLED=n")
endif()

file(WRITE "${sdkconfig_owner_file}" "${sdkconfig_board_key}")

# Fixed-address section for the native library C interface table. The address
# is target-specific and must match VESC_IF in main/c_libs/vesc_c_if.h.
idf_build_get_property(LIBIF_IDF_TARGET IDF_TARGET)
set(LIBIF_LD "${CMAKE_CURRENT_SOURCE_DIR}/main/linker_libif_${LIBIF_IDF_TARGET}.ld")
if(NOT EXISTS ${LIBIF_LD})
    message(FATAL_ERROR "No native lib interface linker script for target ${LIBIF_IDF_TARGET} (expected ${LIBIF_LD})")
endif()
target_link_libraries(${PROJECT_NAME}.elf PRIVATE "-T${LIBIF_LD}")
set_property(TARGET ${PROJECT_NAME}.elf APPEND PROPERTY LINK_DEPENDS ${LIBIF_LD})

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
