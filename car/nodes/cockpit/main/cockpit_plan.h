#pragma once

#include "lbr/node.h"

/* Acquisition slots of the cockpit: only its own battery (part TBD). */
enum { COCKPIT_SLOT_BATTERY, COCKPIT_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_cockpit;
