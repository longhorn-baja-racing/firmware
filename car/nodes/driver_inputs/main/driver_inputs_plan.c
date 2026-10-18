#include "driver_inputs_plan.h"

#include "lbr/nodes.h"

/*
 * driver_inputs node: Throttle, brake pressures, steering angle, brake switch.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_pedals[] = {
    LBR_SIG_THROTTLE_POS,
    LBR_SIG_BRAKE_PRESS_FRONT,
    LBR_SIG_BRAKE_PRESS_REAR,
};
static const lbr_signal_id_t k_steering[] = {
    LBR_SIG_STEERING_ANGLE,
};
static const lbr_signal_id_t k_brake_switch[] = {
    LBR_SIG_BRAKE_SWITCH,
};

static const lbr_slot_t k_slots[DRIVER_INPUTS_SLOT_COUNT] = {
    [DRIVER_INPUTS_SLOT_PEDALS] =
        {
            .name        = "pedals",
            .signals     = k_pedals,
            .n_signals   = sizeof k_pedals / sizeof k_pedals[0],
            .rate_hz     = 100,
            .publish_div = 1,
        },
    [DRIVER_INPUTS_SLOT_STEERING] =
        {
            .name        = "steering",
            .signals     = k_steering,
            .n_signals   = sizeof k_steering / sizeof k_steering[0],
            .rate_hz     = 100,
            .publish_div = 1,
        },
    [DRIVER_INPUTS_SLOT_BRAKE_SWITCH] =
        {
            .name        = "brake_switch",
            .signals     = k_brake_switch,
            .n_signals   = sizeof k_brake_switch / sizeof k_brake_switch[0],
            .rate_hz     = 50,
            .publish_div = 1,
        },
};

const lbr_node_plan_t lbr_plan_driver_inputs = {
    .node_id          = LBR_NODE_DRIVER_INPUTS,
    .slots            = k_slots,
    .n_slots          = DRIVER_INPUTS_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
