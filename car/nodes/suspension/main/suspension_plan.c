#include "suspension_plan.h"

#include "lbr/nodes.h"

/*
 * suspension node: Suspension travel at each corner.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_travel[] = {
    LBR_SIG_SUSP_TRAVEL_FL,
    LBR_SIG_SUSP_TRAVEL_FR,
    LBR_SIG_SUSP_TRAVEL_RL,
    LBR_SIG_SUSP_TRAVEL_RR,
};

static const lbr_slot_t k_slots[SUSPENSION_SLOT_COUNT] = {
    [SUSPENSION_SLOT_TRAVEL] =
        {
            .name        = "travel",
            .signals     = k_travel,
            .n_signals   = sizeof k_travel / sizeof k_travel[0],
            .rate_hz     = 200,
            .publish_div = 1,
        },
};

const lbr_node_plan_t lbr_plan_suspension = {
    .node_id          = LBR_NODE_SUSPENSION,
    .slots            = k_slots,
    .n_slots          = SUSPENSION_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
