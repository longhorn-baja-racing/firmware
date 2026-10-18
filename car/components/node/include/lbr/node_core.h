#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/health.h"
#include "lbr/logging.h"
#include "lbr/node.h"
#include "lbr/timebase.h"

/*
 * Sensor-node logic, independent of the platform.
 *
 * Everything a sensor node decides (validation, acquisition and retries,
 * stamping, sequencing, decimation, publishing, local logging, time sync,
 * heartbeats, events) lives here. It touches hardware only through
 * lbr_node_hal_t, so the same code runs on an ESP32 (node_esp.c: FreeRTOS
 * tasks, TWAI, esp_timer, SD) and in the simulator (dev/sim), where every
 * HAL call is shown as a hardware command.
 *
 * Threading: on target, each acquisition slot, CAN RX, and the heartbeat run
 * in their own tasks. Per-slot state is touched only by that slot's caller;
 * shared counters are atomic; event streams are guarded by hal.lock.
 */

typedef enum {
    LBR_LOG_LVL_ERROR = 0,
    LBR_LOG_LVL_WARN,
    LBR_LOG_LVL_INFO,
} lbr_log_level_t;

typedef struct {
    void *ctx;

    /* Clocks. mono_us is the node's raw local clock; the others are the
     * disciplined shared time base (lbr/timebase.h). */
    uint64_t (*mono_us)(void *ctx);
    uint64_t (*now_us)(void *ctx);
    uint64_t (*local_to_shared)(void *ctx, uint64_t local_us);
    bool (*time_synced)(void *ctx);
    lbr_time_update_t (*time_apply)(void *ctx, uint64_t gateway_us, uint64_t local_rx_us,
                                    uint16_t epoch);
    void (*time_servo)(void *ctx, lbr_time_servo_t *out);

    /* CAN controller. send is non-blocking; failures are counted by the
     * controller and reported through can_stats. */
    bool (*can_send)(void *ctx, const lbr_can_frame_t *f);
    void (*can_stats)(void *ctx, lbr_can_stats_t *out);

    /* Local SD card present (redundancy nodes; may be NULL elsewhere). */
    bool (*sd_present)(void *ctx);

    /* Guards the event streams. May be NULL on single-threaded platforms. */
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);

    /* One line of diagnostic text (no newline). */
    void (*log)(void *ctx, lbr_log_level_t level, const char *text);
} lbr_node_hal_t;

/* Facts about this boot that the platform knows. */
typedef struct {
    lbr_reset_reason_t reset_reason;
    uint32_t           boot_count;
    uint32_t           can_bitrate;
    uint32_t           can_budget_permille;
} lbr_node_boot_t;

#define LBR_NODE_SENSOR_RETRY_US 2000000u

typedef struct {
    lbr_stream_t streams[LBR_SENSOR_MAX_SIGNALS];
    atomic_bool  healthy;
    atomic_bool  calibrated;
    uint64_t     next_retry_us; /* local clock */
} lbr_node_slot_state_t;

typedef struct {
    const lbr_node_config_t *cfg;
    lbr_node_hal_t           hal;
    lbr_node_boot_t          boot;
    lbr_logger_t            *log; /* local log, or NULL */

    bool     config_valid;
    bool     can_up;
    bool     sim;
    uint32_t config_crc;
    char     config_text[LBR_NODE_CONFIG_TEXT_MAX];
    char     config_error[128];

    lbr_node_slot_state_t slot[LBR_NODE_MAX_SENSORS];
    lbr_stream_t          evt_stream;
    lbr_stream_t          fault_stream;
    lbr_time_sync_rx_t    sync; /* touched only by the CAN RX caller */

    atomic_uint   acquired;
    atomic_uint   published;
    atomic_uint   logged;
    atomic_uint   overruns;
    atomic_ullong last_sample_us;
    atomic_uint   sync_unmatched;

    /* Heartbeat caller only. */
    uint32_t prev_acquired, prev_published, prev_logged;
    uint32_t last_bus_off;
} lbr_node_core_t;

/*
 * Validate the configuration and build the session config text. Returns true
 * if the configuration may start acquisition; otherwise config_error holds
 * the reason. Call once, before anything else.
 */
bool lbr_node_core_init(lbr_node_core_t *c, const lbr_node_config_t *cfg, const lbr_node_hal_t *hal,
                        const lbr_node_boot_t *boot);

/* After the platform brought up CAN (and the local log, if any): record the
 * outcome and emit the boot events. `log` may be NULL. */
void lbr_node_core_start(lbr_node_core_t *c, bool can_up, bool can_pins_unset, lbr_logger_t *log);

/* First bring-up of slot i's sensor (call from that slot's context). */
void lbr_node_core_slot_start(lbr_node_core_t *c, size_t i);

/* One acquisition period of slot i. `missed` periods were skipped since the
 * last call (counted as overruns). */
void lbr_node_core_slot_tick(lbr_node_core_t *c, size_t i, uint32_t missed);

/* A received CAN frame with its local receive time. Consumes time sync;
 * passes everything else to cfg->on_frame. */
void lbr_node_core_on_frame(lbr_node_core_t *c, const lbr_can_frame_t *f, uint64_t rx_local_us);

/* One heartbeat period: build the record, publish its pages, log it, and
 * raise a bus-off event when the count moved. */
void lbr_node_core_heartbeat(lbr_node_core_t *c, lbr_health_record_t *out);

/* Publish an EVENT or FAULT sample (and log it locally). */
void lbr_node_core_event(lbr_node_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault);

/* Callback shape for lbr_log_service on_event, routed to a core. */
void lbr_node_core_on_log_event(lbr_node_core_t *c, uint16_t code, bool fault);
