#include "driver_input_plan.h"
#include "lbr.h"

/*
 * Driver Input Controller (ESP32, CAN node 3): pedals and steering.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 */

LBR_NODE_MAIN(&lbr_plan_driver_input);
