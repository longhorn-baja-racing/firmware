#include "corner_plan.h"
#include "lbr.h"
#include "sdkconfig.h"

/*
 * Wheel hub node (Mini ESP32-C5, CAN nodes 7..10): the suspension IMU and the hub's
 * own battery. The same firmware runs on all four; CONFIG_LBR_CORNER_POSITION picks the plan.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 *
 * The suspension IMU waits for a part-specific driver (issue #10). Bind it with
 *
 *     static void init(lbr_node *n) { lbr_bind(n, CORNER_SLOT_IMU, &<part>, &state); }
 *     LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION], .init = init);
 */

LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION]);
