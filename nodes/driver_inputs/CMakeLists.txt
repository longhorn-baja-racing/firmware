cmake_minimum_required(VERSION 3.16)

# Shared lbr components plus the settings every node starts from.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig.common;sdkconfig.defaults")
# Optional overlay, e.g. -DLBR_SDKCONFIG_EXTRA=tests/ci/sdkconfig.linkcheck
if(DEFINED LBR_SDKCONFIG_EXTRA)
    list(APPEND SDKCONFIG_DEFAULTS "${LBR_SDKCONFIG_EXTRA}")
endif()
# Build only main and what it depends on.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lbr_driver_inputs)
