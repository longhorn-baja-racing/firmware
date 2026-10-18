#include "temperature_plan.h"

#include "lbr/nodes.h"

/*
 * temperature node: Engine, CVT, gearbox, brake, ambient, and auxiliary temperatures.
 *
 * Pure data, compiled into the host tests: rates and the CAN budget of every
 * node are validated in CI. Rates are starting points; change them here and
 * the budget tests will say whether the bus can carry it.
 */

static const lbr_signal_id_t k_drivetrain[] = {
    LBR_SIG_TEMP_ENGINE,
    LBR_SIG_TEMP_CVT,
    LBR_SIG_TEMP_GEARBOX,
};
static const lbr_signal_id_t k_brakes[] = {
    LBR_SIG_TEMP_BRAKE_FRONT,
    LBR_SIG_TEMP_BRAKE_REAR,
};
static const lbr_signal_id_t k_ambient[] = {
    LBR_SIG_TEMP_AMBIENT,
};
static const lbr_signal_id_t k_aux[] = {
    LBR_SIG_TEMP_AUX_1,
    LBR_SIG_TEMP_AUX_2,
    LBR_SIG_TEMP_AUX_3,
    LBR_SIG_TEMP_AUX_4,
};

static const lbr_slot_t k_slots[TEMPERATURE_SLOT_COUNT] = {
    [TEMPERATURE_SLOT_DRIVETRAIN] =
        {
            .name        = "drivetrain",
            .signals     = k_drivetrain,
            .n_signals   = sizeof k_drivetrain / sizeof k_drivetrain[0],
            .rate_hz     = 10,
            .publish_div = 1,
        },
    [TEMPERATURE_SLOT_BRAKES] =
        {
            .name        = "brakes",
            .signals     = k_brakes,
            .n_signals   = sizeof k_brakes / sizeof k_brakes[0],
            .rate_hz     = 10,
            .publish_div = 1,
        },
    [TEMPERATURE_SLOT_AMBIENT] =
        {
            .name        = "ambient",
            .signals     = k_ambient,
            .n_signals   = sizeof k_ambient / sizeof k_ambient[0],
            .rate_hz     = 1,
            .publish_div = 1,
        },
    [TEMPERATURE_SLOT_AUX] =
        {
            .name        = "aux",
            .signals     = k_aux,
            .n_signals   = sizeof k_aux / sizeof k_aux[0],
            .rate_hz     = 2,
            .publish_div = 1,
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
