#pragma once

#include "lbr/node.h"

/* Acquisition slots of the dashboard node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum { DASHBOARD_SLOT_BUTTONS, DASHBOARD_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_dashboard;

/* Values the cockpit display shows (any source node). Shared with the
 * simulator so its cockpit view is the firmware's. */
extern const lbr_signal_id_t lbr_dashboard_watch[];
extern const size_t          lbr_dashboard_watch_count;
