#pragma once

#include <stdint.h>

#include "lvgl.h"

#define COCKPIT_GRAPHICS_STRESS_COUNT 8
#define COCKPIT_GRAPHICS_BRAKE_COUNT  4

typedef struct {
    float engine_rpm;
    float rpm_max;
    float speed_mps;
    float cvt_temp_c;
    float brake_temp_c[COCKPIT_GRAPHICS_BRAKE_COUNT];
    float fuel_percent;
    float stress[COCKPIT_GRAPHICS_STRESS_COUNT];
    float stress_full_scale;
    uint16_t valid_mask;
} cockpit_graphics_data_t;

enum {
    COCKPIT_GRAPHICS_VALID_RPM       = 1u << 0,
    COCKPIT_GRAPHICS_VALID_SPEED     = 1u << 1,
    COCKPIT_GRAPHICS_VALID_CVT_TEMP  = 1u << 2,
    COCKPIT_GRAPHICS_VALID_BRAKE_FL  = 1u << 3,
    COCKPIT_GRAPHICS_VALID_BRAKE_FR  = 1u << 4,
    COCKPIT_GRAPHICS_VALID_BRAKE_BL  = 1u << 5,
    COCKPIT_GRAPHICS_VALID_BRAKE_BR  = 1u << 6,
    COCKPIT_GRAPHICS_VALID_FUEL      = 1u << 7,
    COCKPIT_GRAPHICS_VALID_STRESS_1  = 1u << 8,
};

/* Create the dashboard inside parent. It keeps a centered 4:3 viewport and
 * recomputes its size whenever the parent changes size. */
lv_obj_t *cockpit_graphics_create(lv_obj_t *parent);
void cockpit_graphics_update(const cockpit_graphics_data_t *data);
