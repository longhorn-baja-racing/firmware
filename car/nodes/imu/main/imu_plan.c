#include "imu_plan.h"

#include "lbr/nodes.h"

/*
 * imu node, the IMU Controller (ESP32-C5, CAN node 1): the chassis IMU wired
 * to it (DAQ-WIRING.md section 4). The four suspension IMUs are on their own
 * wheel hub nodes (car/nodes/corner).
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
 */

static const lbr_signal_id_t k_chassis[] = {
    LBR_SIG_IMU_CH_ACCEL_X, LBR_SIG_IMU_CH_ACCEL_Y, LBR_SIG_IMU_CH_ACCEL_Z,
    LBR_SIG_IMU_CH_GYRO_X,  LBR_SIG_IMU_CH_GYRO_Y,  LBR_SIG_IMU_CH_GYRO_Z,
};

static const lbr_slot_t k_slots[IMU_SLOT_COUNT] = {
    [IMU_SLOT_CHASSIS] =
        {
            .name      = "chassis",
            .signals   = k_chassis,
            .n_signals = sizeof k_chassis / sizeof k_chassis[0],
            .rate_hz   = 100,
        },
};

const lbr_node_plan_t lbr_plan_imu = {
    .node_id          = LBR_NODE_IMU,
    .slots            = k_slots,
    .n_slots          = IMU_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
