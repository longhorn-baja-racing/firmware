#include "powertrain_plan.h"
#include "lbr.h"

/*
 * Powertrain Controller (ESP32, CAN node 2): engine and CVT speed, fuel level.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 */

LBR_NODE_MAIN(&lbr_plan_powertrain);
