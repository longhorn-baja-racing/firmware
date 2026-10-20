#pragma once

#include "lbr/node.h"

/* Acquisition slots of the IMU Controller. The chassis IMU waits for a
 * part-specific driver; corner_status is fed by the corner receiver. */
enum { IMU_SLOT_CHASSIS, IMU_SLOT_CORNER_STATUS, IMU_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_imu;
