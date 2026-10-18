#include "stress_plan.h"

#include "lbr/nodes.h"

/*
 * stress node: Structural strain gauges. High rate: logged locally at full rate, published
 * to CAN decimated 10:1.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_strain[] = {
    LBR_SIG_STRAIN_1, LBR_SIG_STRAIN_2, LBR_SIG_STRAIN_3, LBR_SIG_STRAIN_4,
    LBR_SIG_STRAIN_5, LBR_SIG_STRAIN_6, LBR_SIG_STRAIN_7, LBR_SIG_STRAIN_8,
};

static const lbr_slot_t k_slots[STRESS_SLOT_COUNT] = {
    [STRESS_SLOT_STRAIN] =
        {
            .name        = "strain",
            .signals     = k_strain,
            .n_signals   = sizeof k_strain / sizeof k_strain[0],
            .rate_hz     = 1000,
            .publish_div = 10,
        },
};

const lbr_node_plan_t lbr_plan_stress = {
    .node_id          = LBR_NODE_STRESS,
    .slots            = k_slots,
    .n_slots          = STRESS_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = true,
    .listen_all       = false,
};
