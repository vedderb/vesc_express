cmake_minimum_required(VERSION 3.5)

# Default build target if not provided
if(NOT DEFINED HW_NAME)
    set(HW_NAME "VESC Express T")
endif()

# Check for buildall target in a different way
if(DEFINED BUILDALL AND BUILDALL)
    # Make sure the build directory exists
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
    
    # Find all available hardware targets
    file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        main/hwconf/hw_*.h
    )
    
    set(ALL_HW_NAMES "")
    foreach(HW_HEADER_PATH ${HW_HEADER_PATHS})
        file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_HEADER_PATH}" FILE_CONTENTS)
        
        foreach(LINE IN LISTS FILE_CONTENTS)
            string(REGEX MATCH "^#define[ \t]+HW_NAME[ \t]+\"(.*)\"$" MATCHES "${LINE}")
            if(MATCHES)
                list(APPEND ALL_HW_NAMES ${CMAKE_MATCH_1})
            endif()
        endforeach()
    endforeach()
    
    message("Found hardware targets, building them all...")
    
    # Create a Python script in the build directory to safely build all targets
    set(BUILD_ALL_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/build/build_all.py")
    file(WRITE ${BUILD_ALL_SCRIPT} "#!/usr/bin/env python3\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "import os\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "import sys\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "import subprocess\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "import platform\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "# Project directory\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "project_dir = '${CMAKE_CURRENT_SOURCE_DIR}'\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "# List of targets to build\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "targets = [\n")
    foreach(TARGET_NAME ${ALL_HW_NAMES})
        file(APPEND ${BUILD_ALL_SCRIPT} "    '${TARGET_NAME}',\n")
    endforeach()
    file(APPEND ${BUILD_ALL_SCRIPT} "]\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "# Clear main build cache to avoid conflicts\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "cache_file = os.path.join(project_dir, 'build', 'CMakeCache.txt')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "if os.path.exists(cache_file):\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    os.remove(cache_file)\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "cache_dir = os.path.join(project_dir, 'build', 'CMakeFiles')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "if os.path.exists(cache_dir):\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    import shutil\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    shutil.rmtree(cache_dir)\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "success = True\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "for target in targets:\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    print(f'\\nBuilding target: {target}')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    # Create exact target directory name without any added characters\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    build_dir = os.path.join(project_dir, 'build', target)\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    os.makedirs(build_dir, exist_ok=True)\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "    # Prepare command arguments as a list (preserves spaces properly)\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    cmd = ['idf.py']\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    cmd.extend(['-B', build_dir])\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    cmd.append(f'-DHW_NAME={target}')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    cmd.append('build')\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "    print(f'Command: {\" \".join(cmd)}')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    # Run the build process\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    result = subprocess.run(cmd, cwd=project_dir)\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    if result.returncode != 0:\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "        success = False\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "        print(f'Failed to build target: {target}')\n\n")
    
    file(APPEND ${BUILD_ALL_SCRIPT} "if success:\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    print('All targets built successfully!')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    sys.exit(0)\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "else:\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    print('Some targets failed to build. Check the build logs for details.')\n")
    file(APPEND ${BUILD_ALL_SCRIPT} "    sys.exit(1)\n")
    
    # Make the script executable on Unix-like systems
    if(NOT CMAKE_HOST_WIN32)
        execute_process(COMMAND chmod +x ${BUILD_ALL_SCRIPT})
    endif()
    
    # Run the Python script
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env python3 ${BUILD_ALL_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE BUILD_ALL_RESULT
    )
    
    # Report result
    if(BUILD_ALL_RESULT EQUAL 0)
        message("All targets built successfully!")
    else()
        message(FATAL_ERROR "Some targets failed to build. Check the build logs for details.")
    endif()
    
    # Stop CMake processing since we've handled all builds
    return()
endif()

# For single target build, set build directory to be target-specific
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
