#pragma once

#include "lbr/node.h"

/* The Central Controller's own acquisition slots: the GPS receiver and the
 * dashboard link status. */
enum { CENTRAL_SLOT_GPS, CENTRAL_SLOT_DASH_STATUS, CENTRAL_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_central;
