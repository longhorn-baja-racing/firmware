#pragma once

#include <stdint.h>

/*
 * Stable signal identifiers (schema-driven protocol).
 *
 * These values are part of the wire/log/CAN protocol and MUST stay stable and
 * synchronized with lbr-dashboard. APPEND ONLY: never renumber or reuse an id.
 * Specific sensor parts are not chosen yet; these are the signal *categories*
 * the 2026 vehicle intends to produce. Units/types are metadata in the shared
 * spec, not implied by the id.
 */
typedef enum {
    LBR_SIG_UNKNOWN = 0x0000,

    /* IMU / inertial (sensor-frame; mounting transform applied by dashboard) */
    LBR_SIG_ACCEL_X = 0x0100,
    LBR_SIG_ACCEL_Y,
    LBR_SIG_ACCEL_Z,
    LBR_SIG_GYRO_X,
    LBR_SIG_GYRO_Y,
    LBR_SIG_GYRO_Z,
    LBR_SIG_ORIENT_QUAT_W,
    LBR_SIG_ORIENT_QUAT_X,
    LBR_SIG_ORIENT_QUAT_Y,
    LBR_SIG_ORIENT_QUAT_Z,
    LBR_SIG_IMU_TEMP,
    LBR_SIG_IMU_CALIB_STATE,

    /* Powertrain / speed */
    LBR_SIG_ENGINE_RPM = 0x0200,
    LBR_SIG_VEHICLE_SPEED,
    LBR_SIG_WHEEL_SPEED_FL,
    LBR_SIG_WHEEL_SPEED_FR,
    LBR_SIG_WHEEL_SPEED_RL,
    LBR_SIG_WHEEL_SPEED_RR,
    LBR_SIG_FUEL_LEVEL,

    /* Temperatures */
    LBR_SIG_TEMP_ENGINE = 0x0300,
    LBR_SIG_TEMP_CVT,
    LBR_SIG_TEMP_BRAKE_FRONT,
    LBR_SIG_TEMP_BRAKE_REAR,
    LBR_SIG_TEMP_AMBIENT,

    /* Electrical */
    LBR_SIG_BATT_VOLTAGE = 0x0400,
    LBR_SIG_BATT_CURRENT,

    /* Events / status (string/enumerated in the shared spec) */
    LBR_SIG_EVENT = 0x0700,
    LBR_SIG_FAULT,
} lbr_signal_id_t;
