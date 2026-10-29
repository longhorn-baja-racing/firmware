#include "central_plan.h"

#include "lbr/nodes.h"

/*
 * central node, the Central Controller (ESP32-C5, CAN node 6): logger, time
 * master, LoRa telemetry, the dashboard's WiFi link, and the GPS receiver
 * (DAQ-WIRING.md section 4). These slots are its own measurements; they go
 * on CAN in its frames (0x600..0x60A) and so into the log like any other.
 *
 * Pure data, compiled into the host tests (see the other plans).
 */

static const lbr_signal_id_t k_gps[] = {
    LBR_SIG_GPS_LATITUDE,   LBR_SIG_GPS_LONGITUDE, LBR_SIG_GPS_ALTITUDE,
    LBR_SIG_GPS_SPEED,      LBR_SIG_GPS_COURSE,    LBR_SIG_GPS_FIX,
    LBR_SIG_GPS_SATELLITES, LBR_SIG_GPS_HDOP,      LBR_SIG_GPS_UTC,
};
static const lbr_signal_id_t k_dash_status[] = {LBR_SIG_DASH_BATTERY, LBR_SIG_DASH_LINK};

static const lbr_slot_t k_slots[CENTRAL_SLOT_COUNT] = {
    [CENTRAL_SLOT_GPS] =
        {
            .name      = "gps",
            .signals   = k_gps,
            .n_signals = sizeof k_gps / sizeof k_gps[0],
            .rate_hz   = 10,
        },
    [CENTRAL_SLOT_DASH_STATUS] =
        {
            .name      = "dash_status",
            .signals   = k_dash_status,
            .n_signals = sizeof k_dash_status / sizeof k_dash_status[0],
            .rate_hz   = 1,
        },
};

const lbr_node_plan_t lbr_plan_central = {
    .node_id          = LBR_NODE_CENTRAL,
    .slots            = k_slots,
    .n_slots          = CENTRAL_SLOT_COUNT,
    .health_period_ms = 1000,
    .local_log        = false,
    .listen_all       = true,
};
