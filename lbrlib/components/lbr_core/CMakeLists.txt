# LBRLib layer 2, core (pure C, no ESP-IDF headers): everything the boards do,
# as logic the host tests and the simulator run unchanged.
#   sensors/  driver kit, configurable inputs, NMEA parsing, the simulator
#   node/     acquisition, plans and validation, the sensor-node core, tables
#   central/  the Central's core: gateway, config, status, LoRa policy
#   cockpit/  the cockpit link and the cockpit core
#   update/   firmware update over CAN
#   logic/    values with status, car logic tasks and outputs
set(dirs sensors node central cockpit update logic)
set(exclude "")
# The bench simulator is compiled only into bench builds, never a race build.
if(NOT CONFIG_LBR_SENSOR_SIM)
    list(APPEND exclude "sensors/sensor_sim.c")
endif()
idf_component_register(
    SRC_DIRS ${dirs}
    EXCLUDE_SRCS ${exclude}
    INCLUDE_DIRS "include"
    REQUIRES lbr_base
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
