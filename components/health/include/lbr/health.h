#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Heartbeat / status record. Emitted on a fixed cadence even when measurements
 * are unchanged. Mirrors the required health set in the shared spec. Counters
 * never silently reset - rollover behavior is defined in the protocol.
 */
typedef struct {
    uint32_t fw_version;
    uint32_t protocol_version;
    uint32_t node_id;

    uint64_t uptime_us;
    uint32_t reset_reason; /* watchdog / brownout / power-on - observable next session */

    /* rates */
    uint32_t acquire_hz;
    uint32_t publish_hz;
    uint32_t log_hz;

    /* queue pressure / losses - never silently discard; count everything */
    uint32_t queue_high_water;
    uint32_t overflow_count;
    uint32_t write_error_count;
    uint32_t transport_error_count;

    /* CAN (TWAI) bus health */
    uint32_t can_tx_err;
    uint32_t can_rx_err;
    uint32_t can_bus_off_count;
    uint32_t can_dropped_frames;

    /* sensor / storage */
    uint32_t sensor_status;
    uint32_t calibration_state;
    uint64_t last_sample_age_us;
    uint64_t last_sd_write_age_us;
    bool     sd_present;
} lbr_health_record_t;
