#include "corner_plan.h"

#include "lbr/nodes.h"

/*
 * corner nodes, the wheel hub controllers (Mini ESP32-C5, CAN nodes 7..10):
 * one suspension IMU each, sent onto CAN in that corner's IMU frames
 * (DAQ-WIRING.md section 4).
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
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

#define SLOTS(i)                                                                                   \
    {                                                                                              \
        [CORNER_SLOT_IMU] = {.name = "imu", .signals = k_imu[i], .n_signals = 6, .rate_hz = 100},  \
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
