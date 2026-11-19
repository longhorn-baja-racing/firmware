#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/cockpit_core.h"
#include "lbr/node.h"
#include "lbr/value_table.h"

/*
 * Writing the cockpit's firmware (car/nodes/cockpit/main/main.c): the
 * driver's display, on its own battery, off the CAN bus. The whole app is:
 *
 *     #include "cockpit_plan.h"     // generated from car/catalog.toml
 *     #include "lbr.h"
 *     #include "panel.h"            // your display driver
 *
 *     LBR_COCKPIT_MAIN(&lbr_plan_cockpit, .display = &panel);
 *
 * The role joins the Central's access point, receives what the Central
 * sends (live values and node status), reports the cockpit's own slots (its
 * battery) back, and calls the display driver to draw. All decisions are in
 * the pure cockpit core (lbr/cockpit_core.h), which the simulator runs.
 */

/* A display driver: how one panel shows the cockpit's view. */
typedef struct {
    const char *panel;   /* panel name, for the log */
    void (*start)(void); /* bring the panel up (once) */
    /* Draw: the latest value per display signal (lbr_value_table_get gives
     * each one's state: OK, STALE, INVALID, NONE), the CAN nodes' status, and
     * whether the Central is heard. Runs in its own task: drawing may be slow. */
    void (*render)(const lbr_value_table_t *values,
                   const lbr_cockpit_node_t nodes[LBR_COCKPIT_MAX_NODES], bool link_up,
                   uint64_t now_us);
    uint32_t period_ms; /* how often to draw (0: 500 ms) */
} lbr_display;

typedef struct {
    const lbr_node_plan_t *plan;    /* generated: lbr_plan_cockpit */
    const lbr_display     *display; /* NULL: no panel (values still reach the log) */
} lbr_cockpit_app_t;

#define LBR_COCKPIT_MAIN(plan_, ...)                                                               \
    void app_main(void);                                                                           \
    void app_main(void)                                                                            \
    {                                                                                              \
        static const lbr_cockpit_app_t lbr_app_ = {.plan = (plan_), __VA_ARGS__};                  \
        lbr_cockpit_main(&lbr_app_);                                                               \
    }                                                                                              \
    static_assert(true, "LBR_COCKPIT_MAIN takes a semicolon")

/* Join the Central, start the slots and the display, and run. Never returns. */
[[noreturn]] void lbr_cockpit_main(const lbr_cockpit_app_t *app);
