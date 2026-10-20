#include "corner_plan.h"

#include "lbr/nodes.h"

/*
 * corner nodes (Mini ESP32-C5 on the moving suspension, own battery): one
 * suspension IMU each, sent over 5 GHz WiFi to the IMU Controller, which
 * forwards it onto CAN (DAQ-WIRING.md sections 4 and 8).
 *
 * Pure data, compiled into the host tests: each signal must be in the CAN
 * frame table at the slot's rate (the IMU Controller sends the frames).
 */

#define IMU_SIGNALS(p)                                                                             \
    {LBR_SIG_##p##_ACCEL_X, LBR_SIG_##p##_ACCEL_Y, LBR_SIG_##p##_ACCEL_Z,                          \
     LBR_SIG_##p##_GYRO_X,  LBR_SIG_##p##_GYRO_Y,  LBR_SIG_##p##_GYRO_Z}

static const lbr_signal_id_t k_imu[4][6] = {
    IMU_SIGNALS(IMU_FL),
    IMU_SIGNALS(IMU_FR),
    IMU_SIGNALS(IMU_BL),
    IMU_SIGNALS(IMU_BR),
};
static const lbr_signal_id_t k_battery[4][1] = {
    {LBR_SIG_CORNER_FL_BATTERY},
    {LBR_SIG_CORNER_FR_BATTERY},
    {LBR_SIG_CORNER_BL_BATTERY},
    {LBR_SIG_CORNER_BR_BATTERY},
};

#define SLOTS(i)                                                                                   \
    {                                                                                              \
        [CORNER_SLOT_IMU] = {.name = "imu", .signals = k_imu[i], .n_signals = 6, .rate_hz = 100},  \
        [CORNER_SLOT_BATTERY] = {.name      = "battery",                                           \
                                 .signals   = k_battery[i],                                        \
                                 .n_signals = 1,                                                   \
                                 .rate_hz   = 1},                                                  \
    }

static const lbr_slot_t k_slots[4][CORNER_SLOT_COUNT] = {SLOTS(0), SLOTS(1), SLOTS(2), SLOTS(3)};

#define PLAN(i, id)                                                                                \
    {.node_id          = (id),                                                                     \
     .slots            = k_slots[i],                                                               \
     .n_slots          = CORNER_SLOT_COUNT,                                                        \
     .health_period_ms = 1000,                                                                     \
     .local_log        = false,                                                                    \
     .listen_all       = false}

const lbr_node_plan_t lbr_plan_corner[4] = {
    PLAN(0, LBR_NODE_CORNER_FL),
    PLAN(1, LBR_NODE_CORNER_FR),
    PLAN(2, LBR_NODE_CORNER_BL),
    PLAN(3, LBR_NODE_CORNER_BR),
};
