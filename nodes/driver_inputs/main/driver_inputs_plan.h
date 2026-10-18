#pragma once

#include "lbr/node.h"

/* Acquisition slots of the driver_inputs node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum {
    DRIVER_INPUTS_SLOT_PEDALS,
    DRIVER_INPUTS_SLOT_STEERING,
    DRIVER_INPUTS_SLOT_BRAKE_SWITCH,
    DRIVER_INPUTS_SLOT_COUNT
};

extern const lbr_node_plan_t lbr_plan_driver_inputs;
