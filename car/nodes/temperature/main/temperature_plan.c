#include "temperature_plan.h"

#include "lbr/nodes.h"

/*
 * temperature node, the Temperature Controller (ESP32, CAN node 4): four brake temperatures and the
 * CVT temperature (DAQ-WIRING.md section 4).
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
 */

static const lbr_signal_id_t k_brakes[] = {LBR_SIG_TEMP_BRAKE_FL, LBR_SIG_TEMP_BRAKE_FR,
                                           LBR_SIG_TEMP_BRAKE_BL, LBR_SIG_TEMP_BRAKE_BR};
static const lbr_signal_id_t k_cvt[]    = {LBR_SIG_TEMP_CVT};

static const lbr_slot_t k_slots[TEMPERATURE_SLOT_COUNT] = {
    [TEMPERATURE_SLOT_BRAKES] =
        {
            .name      = "brakes",
            .signals   = k_brakes,
            .n_signals = sizeof k_brakes / sizeof k_brakes[0],
            .rate_hz   = 10,
        },
    [TEMPERATURE_SLOT_CVT] =
        {
            .name      = "cvt",
            .signals   = k_cvt,
            .n_signals = sizeof k_cvt / sizeof k_cvt[0],
            .rate_hz   = 10,
        },
};

const lbr_node_plan_t lbr_plan_temperature = {
    .node_id          = LBR_NODE_TEMPERATURE,
    .slots            = k_slots,
    .n_slots          = TEMPERATURE_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
