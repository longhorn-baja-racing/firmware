cmake_minimum_required(VERSION 3.16)
project(lbr_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()

set(LBR_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../../components")

# Pure, hardware-independent logic compiled directly for the host. Keep these
# source files free of ESP-IDF includes so they build here and on-target.
add_library(lbr_core
    "${LBR_COMPONENTS}/protocol/protocol.c"
    "${LBR_COMPONENTS}/timebase/timebase.c"
    "${LBR_COMPONENTS}/health/health.c"
)
target_include_directories(lbr_core PUBLIC
    "${LBR_COMPONENTS}/protocol/include"
    "${LBR_COMPONENTS}/timebase/include"
    "${LBR_COMPONENTS}/health/include"
    "${LBR_COMPONENTS}/logging/include"
    "${LBR_COMPONENTS}/transport/include"
    "${LBR_COMPONENTS}/sensors/include"
)
target_compile_options(lbr_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(test_protocol test_protocol.c)
target_link_libraries(test_protocol PRIVATE lbr_core)
add_test(NAME protocol COMMAND test_protocol)
