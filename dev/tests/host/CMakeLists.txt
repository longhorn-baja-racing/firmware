cmake_minimum_required(VERSION 3.16)
project(lbr_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

get_filename_component(LBR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(LBR_COMPONENTS "${LBR_ROOT}/car/components")
include("${LBR_ROOT}/car/cmake/warnings.cmake")

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
    "${LBR_COMPONENTS}/logging/logging.c"
    "${LBR_COMPONENTS}/logging/log_keeper.c"
    "${LBR_COMPONENTS}/sensors/sensor_unconfigured.c"
    "${LBR_COMPONENTS}/sensors/sensor_sim.c"
    "${LBR_COMPONENTS}/sensors/sensor_io.c"
    "${LBR_COMPONENTS}/sensors/nmea.c"
    "${LBR_COMPONENTS}/node/node_config.c"
    "${LBR_COMPONENTS}/node/node_core.c"
    "${LBR_COMPONENTS}/node/node_table.c"
    "${LBR_COMPONENTS}/transport/packet.c"
    "${LBR_COMPONENTS}/transport/lora_policy.c"
    "${LBR_COMPONENTS}/gateway/gw_core.c"
    "${LBR_COMPONENTS}/gateway/gw_config.c"
    "${LBR_COMPONENTS}/gateway/status.c"
    port_host.c
)
target_include_directories(lbr_core PUBLIC
    "${LBR_COMPONENTS}/util/include"
    "${LBR_COMPONENTS}/protocol/include"
    "${LBR_COMPONENTS}/health/include"
    "${LBR_COMPONENTS}/can/include"
    "${LBR_COMPONENTS}/timebase/include"
    "${LBR_COMPONENTS}/logging/include"
    "${LBR_COMPONENTS}/sensors/include"
    "${LBR_COMPONENTS}/node/include"
    "${LBR_COMPONENTS}/transport/include"
    "${LBR_COMPONENTS}/gateway/include"
    "${CMAKE_CURRENT_LIST_DIR}"
)
target_link_libraries(lbr_core PUBLIC m)
lbr_apply_warnings(lbr_core)

if(LBR_SANITIZE AND NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(lbr_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer
                                           -fno-sanitize-recover=all)
    target_link_options(lbr_core PUBLIC -fsanitize=address,undefined)
endif()

target_sources(lbr_core PRIVATE "${LBR_COMPONENTS}/node/value_table.c")

# Every sensor node's plan (pure data) is compiled in, so per-node and
# system-wide CAN budgets are checked against the real configurations.
set(LBR_NODE_PLANS imu_gps suspension driver_inputs temperature fuel_gauge stress dashboard)
add_library(lbr_plans STATIC)
foreach(node IN LISTS LBR_NODE_PLANS)
    target_sources(lbr_plans PRIVATE "${LBR_ROOT}/car/nodes/${node}/main/${node}_plan.c")
    target_include_directories(lbr_plans PUBLIC "${LBR_ROOT}/car/nodes/${node}/main")
endforeach()
target_link_libraries(lbr_plans PUBLIC lbr_core)
lbr_apply_warnings(lbr_plans)

set(LBR_TESTS util protocol health can timebase logging sensors node plans transport gateway cores)
foreach(name IN LISTS LBR_TESTS)
    add_executable(test_${name} test_${name}.c)
    target_link_libraries(test_${name} PRIVATE lbr_core lbr_plans)
    lbr_apply_warnings(test_${name})
    add_test(NAME ${name} COMMAND test_${name})
endforeach()

# Golden fixtures shared with lbr-dashboard. CTest checks the committed files
# still match the codecs; the "fixtures" target regenerates them on purpose.
add_executable(gen_fixtures gen_fixtures.c)
target_link_libraries(gen_fixtures PRIVATE lbr_core)
lbr_apply_warnings(gen_fixtures)
add_test(NAME fixtures COMMAND gen_fixtures check "${LBR_ROOT}/dev/tests/fixtures")
add_custom_target(fixtures
    COMMAND gen_fixtures write "${LBR_ROOT}/dev/tests/fixtures"
    DEPENDS gen_fixtures
    COMMENT "Regenerating golden fixtures in dev/tests/fixtures")
