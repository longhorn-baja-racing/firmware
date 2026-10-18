#include "fuel_gauge_plan.h"

#include "lbr/nodes.h"

/*
 * fuel_gauge node: Fuel level and remaining volume.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_fuel[] = {
    LBR_SIG_FUEL_LEVEL,
    LBR_SIG_FUEL_VOLUME,
};

static const lbr_slot_t k_slots[FUEL_GAUGE_SLOT_COUNT] = {
    [FUEL_GAUGE_SLOT_FUEL] =
        {
            .name        = "fuel",
            .signals     = k_fuel,
            .n_signals   = sizeof k_fuel / sizeof k_fuel[0],
            .rate_hz     = 5,
            .publish_div = 1,
        },
};

const lbr_node_plan_t lbr_plan_fuel_gauge = {
    .node_id          = LBR_NODE_FUEL_GAUGE,
    .slots            = k_slots,
    .n_slots          = FUEL_GAUGE_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
