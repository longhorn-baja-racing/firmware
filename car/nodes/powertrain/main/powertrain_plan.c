#include "powertrain_plan.h"

#include "lbr/nodes.h"

/*
 * powertrain node, the Powertrain Controller (ESP32, CAN node 2): fuel level, engine speed, and CVT
 * speed (DAQ-WIRING.md section 4).
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
 */

static const lbr_signal_id_t k_speeds[] = {LBR_SIG_ENGINE_RPM, LBR_SIG_CVT_RPM};
static const lbr_signal_id_t k_fuel[]   = {LBR_SIG_FUEL_LEVEL};

static const lbr_slot_t k_slots[POWERTRAIN_SLOT_COUNT] = {
    [POWERTRAIN_SLOT_SPEEDS] =
        {
            .name      = "speeds",
            .signals   = k_speeds,
            .n_signals = sizeof k_speeds / sizeof k_speeds[0],
            .rate_hz   = 100,
        },
    [POWERTRAIN_SLOT_FUEL] =
        {
            .name      = "fuel",
            .signals   = k_fuel,
            .n_signals = sizeof k_fuel / sizeof k_fuel[0],
            .rate_hz   = 1,
        },
};

const lbr_node_plan_t lbr_plan_powertrain = {
    .node_id          = LBR_NODE_POWERTRAIN,
    .slots            = k_slots,
    .n_slots          = POWERTRAIN_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
