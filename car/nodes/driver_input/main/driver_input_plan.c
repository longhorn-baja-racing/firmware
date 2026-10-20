#include "driver_input_plan.h"

#include "lbr/nodes.h"

/*
 * driver_input node, the Driver Input Controller (ESP32, CAN node 3): brake pedal, gas pedal, and
 * steering angle (DAQ-WIRING.md section 4).
 *
 * Pure data, compiled into the host tests: each slot's signals must travel in
 * frames of the CAN frame table (car/components/protocol/frames.c) that this
 * node sends at the slot's rate, and the whole bus must stay inside its
 * budget. Change a rate in both places and the tests say whether it fits.
 */

static const lbr_signal_id_t k_inputs[] = {LBR_SIG_BRAKE_PEDAL, LBR_SIG_GAS_PEDAL,
                                           LBR_SIG_STEERING_ANGLE};

static const lbr_slot_t k_slots[DRIVER_INPUT_SLOT_COUNT] = {
    [DRIVER_INPUT_SLOT_INPUTS] =
        {
            .name      = "inputs",
            .signals   = k_inputs,
            .n_signals = sizeof k_inputs / sizeof k_inputs[0],
            .rate_hz   = 100,
        },
};

const lbr_node_plan_t lbr_plan_driver_input = {
    .node_id          = LBR_NODE_DRIVER_INPUT,
    .slots            = k_slots,
    .n_slots          = DRIVER_INPUT_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
