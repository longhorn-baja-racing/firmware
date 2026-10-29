# Every node must be built with exactly the ESP-IDF release pinned in
# .idf-version at the repository root. Different IDF versions produce
# different binaries (drivers, WiFi/BLE stacks, linker layout), so a build
# with anything else fails here instead of producing firmware nobody else can
# reproduce. To move the project to a new IDF release, change .idf-version
# (CI picks its container image from the same file) in its own PR.
#
# Include after ESP-IDF's project.cmake and before project().

get_filename_component(_lbr_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
file(STRINGS "${_lbr_root}/.idf-version" _lbr_idf_pin LIMIT_COUNT 1)
string(STRIP "${_lbr_idf_pin}" _lbr_idf_pin)
if(NOT _lbr_idf_pin MATCHES "^v([0-9]+)\\.([0-9]+)(\\.([0-9]+))?$")
    message(FATAL_ERROR "LBR: .idf-version must look like v6.1 or v6.1.2, got '${_lbr_idf_pin}'")
endif()
set(_lbr_patch "${CMAKE_MATCH_4}")
if("${_lbr_patch}" STREQUAL "")
    set(_lbr_patch 0)
endif()
set(_lbr_want "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${_lbr_patch}")
set(_lbr_have "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

if(NOT _lbr_have STREQUAL _lbr_want)
    message(FATAL_ERROR
        "LBR: this project requires ESP-IDF ${_lbr_idf_pin} (${_lbr_want}), but IDF_PATH "
        "($ENV{IDF_PATH}) is ${_lbr_have}. Install and activate ${_lbr_idf_pin} "
        "(see README.md, Build workflow).")
endif()

# A git checkout must sit exactly on the release tag: a development branch or
# local commits report the same version numbers but are not the release.
# Installs without git metadata (release archives, some installers) are
# checked by version numbers only.
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "$ENV{IDF_PATH}/.git")
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "$ENV{IDF_PATH}" rev-parse HEAD
                    RESULT_VARIABLE _lbr_rc OUTPUT_QUIET ERROR_QUIET)
    if(_lbr_rc EQUAL 0)
        execute_process(COMMAND "${GIT_EXECUTABLE}" -C "$ENV{IDF_PATH}" describe --tags --exact-match HEAD
                        RESULT_VARIABLE _lbr_rc OUTPUT_VARIABLE _lbr_tag
                        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT _lbr_rc EQUAL 0 OR NOT _lbr_tag STREQUAL _lbr_idf_pin)
            message(FATAL_ERROR
                "LBR: ESP-IDF at $ENV{IDF_PATH} is not checked out at the release tag "
                "${_lbr_idf_pin} (HEAD is '${_lbr_tag}'). Run: "
                "git -C \"$ENV{IDF_PATH}\" checkout ${_lbr_idf_pin} && "
                "git -C \"$ENV{IDF_PATH}\" submodule update --init --recursive")
        endif()
    endif()
endif()
message(STATUS "LBR: ESP-IDF ${_lbr_idf_pin} (pinned in .idf-version)")
