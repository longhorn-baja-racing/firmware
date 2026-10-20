#pragma once

#include "lbr/node.h"

/* Acquisition slots of the dashboard: only its own battery (part TBD). */
enum { DASHBOARD_SLOT_BATTERY, DASHBOARD_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_dashboard;
