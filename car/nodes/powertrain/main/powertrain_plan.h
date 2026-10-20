#pragma once

#include "lbr/node.h"

/* Acquisition slots of the powertrain node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum { POWERTRAIN_SLOT_SPEEDS, POWERTRAIN_SLOT_FUEL, POWERTRAIN_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_powertrain;
