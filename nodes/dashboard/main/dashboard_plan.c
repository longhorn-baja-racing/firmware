#include "dashboard_plan.h"

#include "lbr/nodes.h"

/*
 * dashboard node: In-cockpit driver display (not the lbr-dashboard desktop app): listens to
 * the whole bus to show live values and node status; publishes its buttons.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_buttons[] = {
    LBR_SIG_DASH_BUTTONS,
};

static const lbr_slot_t k_slots[DASHBOARD_SLOT_COUNT] = {
    [DASHBOARD_SLOT_BUTTONS] =
        {
            .name        = "buttons",
            .signals     = k_buttons,
            .n_signals   = sizeof k_buttons / sizeof k_buttons[0],
            .rate_hz     = 20,
            .publish_div = 1,
        },
};

const lbr_node_plan_t lbr_plan_dashboard = {
    .node_id          = LBR_NODE_DASHBOARD,
    .slots            = k_slots,
    .n_slots          = DASHBOARD_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = true,
};
