#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/cockpit_core.h"
#include "lbr/value_table.h"

/* Cockpit display backend. The selected panel and GPIO map are still TBD,
 * so this backend reports the display state to the console until the board
 * wiring is defined. */
void cockpit_display_init(void);
void cockpit_display_render(const lbr_value_table_t *values,
                            const lbr_cockpit_node_t nodes[LBR_COCKPIT_MAX_NODES], bool link_up,
                            uint64_t now_us);
