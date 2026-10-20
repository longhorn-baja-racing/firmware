#include "stress_plan.h"

#include "lbr/nodes.h"

/*
 * stress node, the Stress Controller (ESP32, CAN node 5): stress sensors 1 to 8 (DAQ-WIRING.md
 * section 4); where each one sits is session config.
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
 */

static const lbr_signal_id_t k_stress[] = {LBR_SIG_STRESS_1, LBR_SIG_STRESS_2, LBR_SIG_STRESS_3,
                                           LBR_SIG_STRESS_4, LBR_SIG_STRESS_5, LBR_SIG_STRESS_6,
                                           LBR_SIG_STRESS_7, LBR_SIG_STRESS_8};

static const lbr_slot_t k_slots[STRESS_SLOT_COUNT] = {
    [STRESS_SLOT_STRESS] =
        {
            .name      = "stress",
            .signals   = k_stress,
            .n_signals = sizeof k_stress / sizeof k_stress[0],
            .rate_hz   = 100,
        },
};

const lbr_node_plan_t lbr_plan_stress = {
    .node_id          = LBR_NODE_STRESS,
    .slots            = k_slots,
    .n_slots          = STRESS_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
