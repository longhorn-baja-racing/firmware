#pragma once

#include "lbr/node.h"

/* Acquisition slots of the suspension node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum { SUSPENSION_SLOT_TRAVEL, SUSPENSION_SLOT_COUNT };

extern const lbr_node_plan_t lbr_plan_suspension;
