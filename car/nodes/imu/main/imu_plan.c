#include "imu_plan.h"

#include "lbr/nodes.h"

/*
 * imu node, the IMU Controller (ESP32-C5, CAN node 1): the chassis IMU wired
 * to it, plus the four corner nodes' IMU data received over 5 GHz WiFi and
 * forwarded onto CAN (DAQ-WIRING.md sections 4 and 8). Only the chassis IMU
 * and the corner status are slots here; forwarded samples keep the corner's
 * own grid index and need no slot.
 *
 * Pure data, compiled into the host tests (see the other plans).
 */

static const lbr_signal_id_t k_chassis[] = {
    LBR_SIG_IMU_CH_ACCEL_X, LBR_SIG_IMU_CH_ACCEL_Y, LBR_SIG_IMU_CH_ACCEL_Z,
    LBR_SIG_IMU_CH_GYRO_X,  LBR_SIG_IMU_CH_GYRO_Y,  LBR_SIG_IMU_CH_GYRO_Z,
};
static const lbr_signal_id_t k_corner_status[] = {
    LBR_SIG_CORNER_FL_BATTERY, LBR_SIG_CORNER_FR_BATTERY, LBR_SIG_CORNER_BL_BATTERY,
    LBR_SIG_CORNER_BR_BATTERY, LBR_SIG_CORNER_LINK,       LBR_SIG_CORNER_SYNCED,
};

static const lbr_slot_t k_slots[IMU_SLOT_COUNT] = {
    [IMU_SLOT_CHASSIS] =
        {
            .name      = "chassis",
            .signals   = k_chassis,
            .n_signals = sizeof k_chassis / sizeof k_chassis[0],
            .rate_hz   = 100,
        },
    [IMU_SLOT_CORNER_STATUS] =
        {
            .name      = "corner_status",
            .signals   = k_corner_status,
            .n_signals = sizeof k_corner_status / sizeof k_corner_status[0],
            .rate_hz   = 1,
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
