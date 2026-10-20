#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/dash_link.h"
#include "lbr/gw_config.h"
#include "lbr/health.h"
#include "lbr/logging.h"
#include "lbr/lora_policy.h"
#include "lbr/node_core.h"
#include "lbr/node_table.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr/value_table.h"

/*
 * Central Controller (gateway / logger) logic, independent of the platform.
 *
 * Every frame on the bus (and every frame the Central sends) goes to the
 * authoritative log first, then to the USB/WiFi stream, the LoRa policy, and
 * the cockpit dashboard's view. The Central is the time master, tracks node
 * presence and health, samples its own GPS (and the dashboard link state)
 * into ordinary data frames, and pushes display data to the dashboard over
 * WiFi. The hardware is reached only through lbr_gw_hal_t, so the same code
 * runs on the ESP32-C5 (car/nodes/central) and in the simulator (dev/sim).
 *
 * Threading: on target, CAN RX, time sync, each acquisition slot, stream
 * flush, LoRa, the dashboard link, and the heartbeat are separate tasks.
 * Shared state is guarded by hal.lock with one of the lock ids below;
 * counters are atomic.
 */

typedef enum {
    LBR_GW_LOCK_STATE = 0, /* node table, decoder, LoRa policy, dashboard link, last heartbeat */
    LBR_GW_LOCK_STREAM,    /* USB/WiFi stream batcher */
    LBR_GW_LOCK_EVENT,     /* event sequence */
    LBR_GW_LOCK_COUNT_
} lbr_gw_lock_t;

typedef struct {
    void *ctx;

    /* Clocks: the Central is the time master, so shared time is its local
     * clock (monotonic guard applied by now_us). */
    uint64_t (*mono_us)(void *ctx);
    uint64_t (*now_us)(void *ctx);
    uint64_t (*local_to_shared)(void *ctx, uint64_t local_us);

    /* CAN controller. sync_arm is called right before TIME_SYNC is queued,
     * so the platform can capture its TX-done time for the FOLLOW_UP. */
    bool (*can_send)(void *ctx, const lbr_can_frame_t *f);
    void (*can_stats)(void *ctx, lbr_can_stats_t *out);
    void (*sync_arm)(void *ctx);

    /* SD card present (authoritative log). */
    bool (*sd_present)(void *ctx);

    /* LoRa radio free and inside its duty budget. */
    bool (*lora_ready)(void *ctx);

    void (*lock)(void *ctx, lbr_gw_lock_t which);
    void (*unlock)(void *ctx, lbr_gw_lock_t which);

    void (*log)(void *ctx, lbr_log_level_t level, const char *text);
} lbr_gw_hal_t;

typedef struct {
    lbr_reset_reason_t reset_reason;
    uint32_t           boot_count;
    uint16_t           epoch;        /* new random timeline id every boot (never 0) */
    bool               config_valid; /* stored Central config passed validation */
    const char        *config_text;  /* session config; must outlive the core */

    /* The Central's own acquisition slots (GPS, dashboard status), from its
     * plan; their sensors must outlive the core. May be empty. */
    const lbr_acq_t *acq;
    size_t           n_acq;
} lbr_gw_boot_t;

/* Live links; any may be NULL (disabled or unavailable). */
typedef struct {
    lbr_transport_t *usb;
    lbr_transport_t *wifi;
    lbr_transport_t *lora;
    lbr_transport_t *ble;
    lbr_transport_t *dash; /* datagrams to the cockpit dashboard */
} lbr_gw_links_t;

#define LBR_GW_HEALTH_PERIOD_MS 1000u
#define LBR_GW_DASH_STALE_US    2000000u

typedef struct {
    lbr_gw_hal_t    hal;
    lbr_gw_config_t cfg;
    lbr_gw_boot_t   boot;
    uint32_t        config_crc;
    bool            can_up;
    bool            sim;
    lbr_logger_t   *log;
    lbr_gw_links_t  links;

    /* Own acquisition (each slot touched only by its caller). */
    lbr_node_slot_state_t slot[LBR_NODE_MAX_SENSORS];

    /* LBR_GW_LOCK_STATE */
    lbr_node_table_t    nodes;
    lbr_bus_decoder_t   decoder;
    lbr_lora_policy_t   lora_policy;
    lbr_value_table_t   dash_values;
    lbr_dash_link_t     dash;
    lbr_health_record_t last_health;

    /* LBR_GW_LOCK_STREAM */
    uint8_t           stream_pkt[LBR_PKT_MAX_SIZE];
    lbr_pkt_builder_t stream;
    bool              stream_open;

    /* LBR_GW_LOCK_EVENT */
    uint32_t evt_seq;

    /* Time-sync caller only. */
    uint8_t sync_seq;

    /* Heartbeat caller only. */
    uint32_t prev_rx, prev_logged, last_bus_off, prev_loss;

    atomic_uint   frames_rx;
    atomic_uint   frames_logged;
    atomic_uint   sync_tx_fail;
    atomic_uint   overruns;
    atomic_uint   out_of_range;
    atomic_uint   dash_sent;
    atomic_ullong last_sample_us;
} lbr_gw_core_t;

void lbr_gw_core_init(lbr_gw_core_t *c, const lbr_gw_hal_t *hal, const lbr_gw_config_t *cfg,
                      const lbr_gw_boot_t *boot);

/* After the platform started the log, the links, and CAN, and bound the
 * drivers of boot->acq: set up the slots, emit the boot events and
 * SESSION_START. `log` is the authoritative logger (may be NULL if it
 * failed). */
void lbr_gw_core_start(lbr_gw_core_t *c, lbr_logger_t *log, const lbr_gw_links_t *links,
                       bool can_up, bool can_pins_unset);

/* A frame received from the bus, with its local receive time. */
void lbr_gw_core_on_rx(lbr_gw_core_t *c, const lbr_can_frame_t *f, uint64_t rx_local_us);

/* Time sync, LBR_TIME_SYNC_HZ times per second: send TIME_SYNC and return
 * its seq; the platform then reports when it finished transmitting (local
 * clock), or that it did not (no ACK, bus-off), with sync_done. */
uint8_t lbr_gw_core_sync_send(lbr_gw_core_t *c);
void    lbr_gw_core_sync_done(lbr_gw_core_t *c, bool sent, uint64_t tx_local_us);

/* Own acquisition slot i (GPS, dashboard status): as for sensor nodes. */
void     lbr_gw_core_slot_start(lbr_gw_core_t *c, size_t i);
void     lbr_gw_core_slot_tick(lbr_gw_core_t *c, size_t i);
uint64_t lbr_gw_core_slot_next_due(lbr_gw_core_t *c, size_t i);

/* Every stream_batch_ms: push the pending USB/WiFi batch. */
void lbr_gw_core_stream_flush(lbr_gw_core_t *c);

/* Every lora_period_ms: build and send one LoRa packet if the radio is ready. */
void lbr_gw_core_lora_tick(lbr_gw_core_t *c);

/* Every dash_period_ms: link timeout and one display datagram. */
void lbr_gw_core_dash_tick(lbr_gw_core_t *c);

/* A datagram from the cockpit dashboard, with its local receive time. */
void lbr_gw_core_on_dash_packet(lbr_gw_core_t *c, const uint8_t *buf, size_t len,
                                uint64_t rx_local_us);

/* The DASH_STATUS driver needs the state lock; these take it. */
void lbr_gw_core_lock_state(void *core);
void lbr_gw_core_unlock_state(void *core);

/* Every LBR_GW_HEALTH_PERIOD_MS: own heartbeat, node expiry. */
void lbr_gw_core_heartbeat(lbr_gw_core_t *c, lbr_health_record_t *out);

/* Central EVENT / FAULT (sent on CAN and logged like any frame). */
void lbr_gw_core_event(lbr_gw_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault);

/* COMMAND frame to every node (lbr_command_t). */
void lbr_gw_core_command(lbr_gw_core_t *c, uint8_t code, const uint8_t args[7]);

/* Status text for the links (STAT, HTTP, BLE). */
size_t lbr_gw_core_status(lbr_gw_core_t *c, const char *log_name, char *buf, size_t cap);
