cmake_minimum_required(VERSION 3.16)
project(lbr_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

set(LBR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(LBR_COMPONENTS "${LBR_ROOT}/components")
include("${LBR_ROOT}/cmake/warnings.cmake")

option(LBR_SANITIZE "Build host tests with ASan + UBSan" ON)

# Pure, hardware-independent logic compiled directly for the host. These
# sources never include ESP-IDF headers; *_esp.c files hold the platform side
# and port_host.c stands in for it here.
add_library(lbr_core
    "${LBR_COMPONENTS}/util/crc32.c"
    "${LBR_COMPONENTS}/util/cobs.c"
    "${LBR_COMPONENTS}/util/ringbuf.c"
    "${LBR_COMPONENTS}/protocol/protocol.c"
    "${LBR_COMPONENTS}/health/health.c"
    "${LBR_COMPONENTS}/can/can.c"
    "${LBR_COMPONENTS}/timebase/timebase.c"
    port_host.c
)
target_include_directories(lbr_core PUBLIC
    "${LBR_COMPONENTS}/util/include"
    "${LBR_COMPONENTS}/protocol/include"
    "${LBR_COMPONENTS}/health/include"
    "${LBR_COMPONENTS}/can/include"
    "${LBR_COMPONENTS}/timebase/include"
    "${CMAKE_CURRENT_LIST_DIR}"
)
target_link_libraries(lbr_core PUBLIC m)
lbr_apply_warnings(lbr_core)

if(LBR_SANITIZE AND NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(lbr_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer
                                           -fno-sanitize-recover=all)
    target_link_options(lbr_core PUBLIC -fsanitize=address,undefined)
endif()

set(LBR_TESTS util protocol health can timebase)
foreach(name IN LISTS LBR_TESTS)
    add_executable(test_${name} test_${name}.c)
    target_link_libraries(test_${name} PRIVATE lbr_core)
    lbr_apply_warnings(test_${name})
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
