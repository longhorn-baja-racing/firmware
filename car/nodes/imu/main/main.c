#include "imu_plan.h"
#include "lbr.h"
#include "sdkconfig.h"

/*
 * IMU Controller (ESP32-C5, CAN node 1): the chassis IMU.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

/* The IMU waits for a part-specific driver (issue #10). Bind it with
 *
 *     static void init(lbr_node *n) { lbr_sensor_<part>(lbr_slot(n, IMU_SLOT_CHASSIS), ...); }
 *     LBR_NODE_MAIN(&lbr_plan_imu, .init = init);
 */
LBR_NODE_MAIN(&lbr_plan_imu);
