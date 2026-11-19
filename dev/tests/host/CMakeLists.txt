cmake_minimum_required(VERSION 3.21)
project(lbr_host_tests C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

get_filename_component(LBR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(LBR_COMPONENTS "${LBR_ROOT}/lbrlib/components")
include("${LBR_ROOT}/lbrlib/cmake/warnings.cmake")

option(LBR_SANITIZE "Build host tests with ASan + UBSan" ON)

# LBRLib's two pure layers (lbr_base, lbr_core), compiled directly for the host:
# every .c in them, exactly as the firmware builds them. They never include
# ESP-IDF headers (dev/ci/check_layers.py); port_host.c stands in for the
# platform side.
file(GLOB_RECURSE LBR_PURE_SOURCES CONFIGURE_DEPENDS
    "${LBR_COMPONENTS}/lbr_base/*.c" "${LBR_COMPONENTS}/lbr_core/*.c")
add_library(lbr_core ${LBR_PURE_SOURCES} port_host.c)
target_include_directories(lbr_core PUBLIC
    "${LBR_COMPONENTS}/lbr_base/include"
    "${LBR_COMPONENTS}/lbr_core/include"
    "${LBR_ROOT}/car/catalog/include"
    "${CMAKE_CURRENT_LIST_DIR}"
)
target_link_libraries(lbr_core PUBLIC m)
lbr_apply_warnings(lbr_core)

if(LBR_SANITIZE AND NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(lbr_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer
                                           -fno-sanitize-recover=all)
    target_link_options(lbr_core PUBLIC -fsanitize=address,undefined)
endif()


# Every node's plan (pure data) is compiled in, so the frame-table rules and
# the system-wide CAN budget are checked against the real configurations.
set(LBR_NODE_PLANS imu powertrain driver_input temperature stress central cockpit corner)
add_library(lbr_plans STATIC)
foreach(node IN LISTS LBR_NODE_PLANS)
    target_sources(lbr_plans PRIVATE "${LBR_ROOT}/car/nodes/${node}/main/${node}_plan.c")
    target_include_directories(lbr_plans PUBLIC "${LBR_ROOT}/car/nodes/${node}/main")
endforeach()
target_link_libraries(lbr_plans PUBLIC lbr_core)
lbr_apply_warnings(lbr_plans)

# The car's logic tasks (car/logic), checked by test_plans and run by the simulator.
file(GLOB LBR_CAR_LOGIC CONFIGURE_DEPENDS "${LBR_ROOT}/car/logic/*.c")
add_library(lbr_car_logic STATIC ${LBR_CAR_LOGIC})
target_include_directories(lbr_car_logic PUBLIC "${LBR_ROOT}/car/logic/include")
target_link_libraries(lbr_car_logic PUBLIC lbr_core)
lbr_apply_warnings(lbr_car_logic)

set(LBR_TESTS util protocol health can timebase logging sensors node plans transport gateway wlink cores
    update value logic)
foreach(name IN LISTS LBR_TESTS)
    add_executable(test_${name} test_${name}.c)
    target_link_libraries(test_${name} PRIVATE lbr_core lbr_plans lbr_car_logic)
    lbr_apply_warnings(test_${name})
    add_test(NAME ${name} COMMAND test_${name})
endforeach()

# Golden fixtures shared with the dashboard app. CTest checks the committed files
# still match the codecs; the "fixtures" target regenerates them on purpose.
add_executable(gen_fixtures gen_fixtures.c)
target_link_libraries(gen_fixtures PRIVATE lbr_core)
lbr_apply_warnings(gen_fixtures)
add_test(NAME fixtures COMMAND gen_fixtures check "${LBR_ROOT}/dev/tests/fixtures")
# session.csv is the reference decoder's CSV export of session.lbr (docs/protocol.md,
# "CSV export"); the decoder self test checks it alongside the binary fixtures.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(LBR_DECODE "${LBR_ROOT}/dev/tools/lbr_decode.py")
add_custom_target(fixtures
    COMMAND gen_fixtures write "${LBR_ROOT}/dev/tests/fixtures"
    COMMAND Python3::Interpreter -I "${LBR_DECODE}" csv "${LBR_ROOT}/dev/tests/fixtures/session.lbr"
            --out "${LBR_ROOT}/dev/tests/fixtures/session.csv"
    DEPENDS gen_fixtures
    COMMENT "Regenerating golden fixtures in dev/tests/fixtures")
add_test(NAME decoder COMMAND Python3::Interpreter -I "${LBR_DECODE}" --selftest)

# The car's catalog: every generated file (signals, nodes, frames, plans, the can-ids.md
# tables) matches car/catalog.toml, and the generator refuses broken catalogs.
add_test(NAME catalog COMMAND Python3::Interpreter -I "${LBR_ROOT}/lbrlib/tools/lbrgen.py"
                              "${LBR_ROOT}/car/catalog.toml" --check)
add_test(NAME lbrgen COMMAND Python3::Interpreter "${LBR_ROOT}/lbrlib/tools/test_lbrgen.py")

# LBRLib stays layered: no component includes a header from a layer above it, and the
# pure layers include nothing but the C library (dev/ci/check_layers.py).
add_test(NAME layers COMMAND Python3::Interpreter -I "${LBR_ROOT}/dev/ci/check_layers.py")
