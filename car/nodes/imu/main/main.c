#include "imu_plan.h"
#include "lbr.h"

/*
 * IMU Controller (ESP32-C5, CAN node 1): the chassis IMU.
 *
 * Everything is the shared node runtime (lbrlib/components/lbr_node); the slots
 * the catalog marks input = "io" bind from menuconfig (main/Kconfig).
 *
 * The chassis IMU waits for a part-specific driver (issue #10). Bind it with
 *
 *     static void init(lbr_node *n) { lbr_bind(n, IMU_SLOT_CHASSIS, &<part>, &state); }
 *     LBR_NODE_MAIN(&lbr_plan_imu, .init = init);
 */

LBR_NODE_MAIN(&lbr_plan_imu);
