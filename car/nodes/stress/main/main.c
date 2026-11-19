#include "stress_plan.h"
#include "lbr.h"

/*
 * Stress Controller (ESP32, CAN node 5): stress sensors 1..8.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 */

LBR_NODE_MAIN(&lbr_plan_stress);
