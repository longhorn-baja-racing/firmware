#pragma once

#include "lbr/node.h"

/* Acquisition slots of the imu_gps node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum {
    IMU_GPS_SLOT_IMU,
    IMU_GPS_SLOT_ORIENT,
    IMU_GPS_SLOT_MAG,
    IMU_GPS_SLOT_IMU_STATUS,
    IMU_GPS_SLOT_GPS,
    IMU_GPS_SLOT_COUNT
};

extern const lbr_node_plan_t lbr_plan_imu_gps;
