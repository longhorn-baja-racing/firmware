#include "imu_gps_plan.h"

#include "lbr/nodes.h"

/*
 * imu_gps node: IMU (acceleration, angular rate, orientation, magnetic field) and GPS.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_imu[] = {
    LBR_SIG_ACCEL_X, LBR_SIG_ACCEL_Y, LBR_SIG_ACCEL_Z,
    LBR_SIG_GYRO_X,  LBR_SIG_GYRO_Y,  LBR_SIG_GYRO_Z,
};
static const lbr_signal_id_t k_imu_orientation[] = {
    LBR_SIG_ORIENT_QUAT_W,
    LBR_SIG_ORIENT_QUAT_X,
    LBR_SIG_ORIENT_QUAT_Y,
    LBR_SIG_ORIENT_QUAT_Z,
};
static const lbr_signal_id_t k_imu_mag[] = {
    LBR_SIG_MAG_X,
    LBR_SIG_MAG_Y,
    LBR_SIG_MAG_Z,
};
static const lbr_signal_id_t k_imu_status[] = {
    LBR_SIG_IMU_TEMP,
    LBR_SIG_IMU_CALIB_STATE,
};
static const lbr_signal_id_t k_gps[] = {
    LBR_SIG_GPS_LATITUDE,   LBR_SIG_GPS_LONGITUDE, LBR_SIG_GPS_ALTITUDE,
    LBR_SIG_GPS_SPEED,      LBR_SIG_GPS_COURSE,    LBR_SIG_GPS_FIX,
    LBR_SIG_GPS_SATELLITES, LBR_SIG_GPS_HDOP,      LBR_SIG_GPS_UTC,
};

static const lbr_slot_t k_slots[IMU_GPS_SLOT_COUNT] = {
    [IMU_GPS_SLOT_IMU] =
        {
            .name        = "imu",
            .signals     = k_imu,
            .n_signals   = sizeof k_imu / sizeof k_imu[0],
            .rate_hz     = 100,
            .publish_div = 1,
        },
    [IMU_GPS_SLOT_ORIENT] =
        {
            .name        = "imu_orientation",
            .signals     = k_imu_orientation,
            .n_signals   = sizeof k_imu_orientation / sizeof k_imu_orientation[0],
            .rate_hz     = 20,
            .publish_div = 1,
        },
    [IMU_GPS_SLOT_MAG] =
        {
            .name        = "imu_mag",
            .signals     = k_imu_mag,
            .n_signals   = sizeof k_imu_mag / sizeof k_imu_mag[0],
            .rate_hz     = 10,
            .publish_div = 1,
        },
    [IMU_GPS_SLOT_IMU_STATUS] =
        {
            .name        = "imu_status",
            .signals     = k_imu_status,
            .n_signals   = sizeof k_imu_status / sizeof k_imu_status[0],
            .rate_hz     = 1,
            .publish_div = 1,
        },
    [IMU_GPS_SLOT_GPS] =
        {
            .name        = "gps",
            .signals     = k_gps,
            .n_signals   = sizeof k_gps / sizeof k_gps[0],
            .rate_hz     = 10,
            .publish_div = 1,
        },
};

const lbr_node_plan_t lbr_plan_imu_gps = {
    .node_id          = LBR_NODE_IMU_GPS,
    .slots            = k_slots,
    .n_slots          = IMU_GPS_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
