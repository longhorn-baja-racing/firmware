#pragma once

#include "lbr/node.h"

/* Acquisition slots of the stress node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum { STRESS_SLOT_STRAIN, STRESS_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_stress;
