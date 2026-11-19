#include "temperature_plan.h"
#include "lbr.h"

/*
 * Temperature Controller (ESP32, CAN node 4): brake and CVT temperatures.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 */

LBR_NODE_MAIN(&lbr_plan_temperature);
