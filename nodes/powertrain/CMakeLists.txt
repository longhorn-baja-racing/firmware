cmake_minimum_required(VERSION 3.16)

# Share the repo's lbr/* components across every node app.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lbr_powertrain)
