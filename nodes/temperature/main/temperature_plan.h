#pragma once

#include "lbr/node.h"

/* Acquisition slots of the temperature node. Bind a real driver to a slot in main.c
 * once its part is chosen; unbound slots report NOT_FITTED. */
enum {
    TEMPERATURE_SLOT_DRIVETRAIN,
    TEMPERATURE_SLOT_BRAKES,
    TEMPERATURE_SLOT_AMBIENT,
    TEMPERATURE_SLOT_AUX,
    TEMPERATURE_SLOT_COUNT
};

extern const lbr_node_plan_t lbr_plan_temperature;
