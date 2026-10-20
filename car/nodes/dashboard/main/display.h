#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/dash_core.h"
#include "lbr/value_table.h"

/*
 * Cockpit display backend. The display hardware is not chosen yet, so the
 * only backend prints a status summary to the console; a real panel driver
 * replaces display.c behind these two calls.
 */
void dash_display_init(void);
void dash_display_render(const lbr_value_table_t *values,
                         const lbr_dash_node_t nodes[LBR_DASH_MAX_NODES], bool link_up,
                         uint64_t now_us);
