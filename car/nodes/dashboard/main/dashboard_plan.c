#include "dashboard_plan.h"

#include "lbr/nodes.h"

/*
 * dashboard node, the Dashboard Controller (ESP32-C5, own battery): the
 * driver's display, fed by the Central over 5 GHz WiFi, with no sensors on
 * the car and no CAN (DAQ-WIRING.md section 4). It reports its battery back;
 * the Central puts it on CAN in DASH_STATUS (0x60A).
 *
 * Pure data, compiled into the host tests (see the other plans).
 */

static const lbr_signal_id_t k_battery[] = {LBR_SIG_DASH_BATTERY};

static const lbr_slot_t k_slots[DASHBOARD_SLOT_COUNT] = {
    [DASHBOARD_SLOT_BATTERY] =
        {
            .name      = "battery",
            .signals   = k_battery,
            .n_signals = sizeof k_battery / sizeof k_battery[0],
            .rate_hz   = 1,
        },
};

const lbr_node_plan_t lbr_plan_dashboard = {
    .node_id          = LBR_NODE_DASHBOARD,
    .slots            = k_slots,
    .n_slots          = DASHBOARD_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = false,
};
