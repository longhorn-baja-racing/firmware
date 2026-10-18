cmake_minimum_required(VERSION 3.16)

# Shared lbr components plus the settings every node starts from.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig.common;sdkconfig.defaults")
# Build only main and what it depends on.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lbr_imu_gps)
