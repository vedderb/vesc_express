function(git_rev_parse _var _repo_dir)
    if(NOT GIT_FOUND)
        find_package(Git QUIET)
    endif()
    if(NOT GIT_FOUND)
        set(${_var} "GIT-NOTFOUND" PARENT_SCOPE)
        return()
    endif()

    execute_process(COMMAND
        "${GIT_EXECUTABLE}"
        "-C"
        ${_repo_dir}
        rev-parse
        ${ARGN}
        WORKING_DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}"
        RESULT_VARIABLE
        res
        OUTPUT_VARIABLE
        out
        ERROR_VARIABLE
        error
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT res EQUAL 0)
        string(STRIP "${error}" error)
        message(STATUS "git rev-parse returned '${error}'")
        set(out "${out}-${res}-NOTFOUND")
    endif()

    set(${_var} "${out}" PARENT_SCOPE)
endfunction()