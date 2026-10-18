#pragma once

#include "lbr/node.h"

/* Acquisition slots of the fuel_gauge node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum { FUEL_GAUGE_SLOT_FUEL, FUEL_GAUGE_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_fuel_gauge;
