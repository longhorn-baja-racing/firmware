#include "corner_plan.h"
#include "lbr.h"
#include "sdkconfig.h"

/*
 * Wheel hub node (Mini ESP32-C5, CAN nodes 7..10): one suspension IMU. The same
 * firmware runs on all four; CONFIG_LBR_CORNER_POSITION picks the plan.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

/* The IMU waits for a part-specific driver (issue #10). Bind it with
 *
 *     static void init(lbr_node *n) { lbr_sensor_<part>(lbr_slot(n, CORNER_SLOT_IMU), ...); }
 *     LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION], .init = init);
 */
LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION]);
