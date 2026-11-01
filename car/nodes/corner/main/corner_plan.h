#pragma once

#include "lbr/node.h"

/* Acquisition slots of a corner node. The suspension IMU waits for a
 * part-specific driver; unbound, it reports NOT_FITTED. */
enum { CORNER_SLOT_IMU, CORNER_SLOT_COUNT };

/* One plan per position: [0] FL, [1] FR, [2] BL, [3] BR. The same firmware
 * runs on all four; CONFIG_LBR_CORNER_POSITION picks the plan. */
extern const lbr_node_plan_t lbr_plan_corner[4];
