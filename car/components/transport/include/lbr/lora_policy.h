#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/health.h"
#include "lbr/protocol.h"

/*
 * In-race LoRa policy. LoRa is the only link assumed live during a race and
 * carries a few hundred bytes per second at best, so it gets a prioritized,
 * downsampled critical subset, built fresh for each radio packet:
 *
 *   1. fault and event samples, oldest first (bounded queue; when full an
 *      event is evicted for a fault, and every drop is counted);
 *   2. compact per-node health (HEALTH_LITE) at a fixed interval per node;
 *   3. the latest value of each subset signal, at most once per its
 *      interval and only when a newer sample arrived (round-robin so no
 *      signal starves).
 *
 * Downsampling here never touches the SD log, which keeps every sample.
 */

#define LBR_LORA_MAX_ITEMS   24u
#define LBR_LORA_EVT_QUEUE   16u
#define LBR_LORA_MAX_NODES   16u
#define LBR_HEALTH_LITE_SIZE 8u

typedef struct {
    lbr_signal_id_t signal;
    uint16_t        interval_ms;
} lbr_lora_item_t;

typedef struct {
    lbr_lora_item_t item;
    bool            have;
    bool            fresh;
    lbr_sample_t    last;
    uint64_t        last_sent_us;
} lbr_lora_slot_t;

typedef struct {
    uint8_t  node_id;
    bool     fresh;
    uint8_t  lite[LBR_HEALTH_LITE_SIZE];
    uint64_t last_sent_us;
} lbr_lora_node_t;

typedef struct {
    lbr_lora_slot_t slots[LBR_LORA_MAX_ITEMS];
    size_t          n_slots;
    size_t          rr; /* round-robin start for signal slots */

    lbr_sample_t events[LBR_LORA_EVT_QUEUE];
    size_t       evt_head;
    size_t       evt_count;

    lbr_lora_node_t nodes[LBR_LORA_MAX_NODES];
    uint32_t        health_interval_ms;

    uint32_t events_dropped;  /* events evicted or rejected: counted, reported */
    uint32_t nodes_untracked; /* health from more nodes than fit */
} lbr_lora_policy_t;

/* The default in-race subset: key temperatures, fuel, battery, position,
 * speed. Faults/events and health are always included. */
extern const lbr_lora_item_t lbr_lora_default_items[];
extern const size_t          lbr_lora_default_item_count;

void lbr_lora_policy_init(lbr_lora_policy_t *p, const lbr_lora_item_t *items, size_t n_items,
                          uint32_t health_interval_ms);

void lbr_lora_policy_offer_sample(lbr_lora_policy_t *p, const lbr_sample_t *s);
void lbr_lora_policy_offer_health(lbr_lora_policy_t *p, const lbr_health_record_t *h);

/* Build the next packet (see packet.h) into `buf`; returns its length or 0
 * if nothing is due. `cap` is normally LBR_PKT_LORA_MAX. */
size_t lbr_lora_policy_build(lbr_lora_policy_t *p, uint64_t now_us, uint8_t source, uint32_t seq,
                             uint8_t *buf, size_t cap);

/* HEALTH_LITE: node_id u8, state u8, flags u16, sensor_status u32. */
void lbr_health_lite_encode(const lbr_health_record_t *h, uint8_t out[LBR_HEALTH_LITE_SIZE]);

/* ------------------------------------------------------------------------ */
/* Airtime / duty cycle                                                      */

typedef struct {
    uint8_t  sf;       /* spreading factor 6..12 */
    uint32_t bw_hz;    /* 125000, 250000, 500000 */
    uint8_t  cr;       /* coding rate 1..4 => 4/5..4/8 */
    uint16_t preamble; /* symbols, typically 8 */
    bool     crc;      /* payload CRC on */
    bool     implicit; /* implicit header mode */
} lbr_lora_modem_t;

/* Time on air of one packet, microseconds (Semtech SX127x formula). Low data
 * rate optimization is applied automatically when a symbol exceeds 16 ms. */
uint32_t lbr_lora_airtime_us(const lbr_lora_modem_t *m, size_t payload_len);

/* Earliest next transmit time that keeps the long-run duty cycle at or
 * below `duty_permille`, after sending a packet of `airtime_us` at `now`. */
uint64_t lbr_lora_next_tx_us(uint64_t now_us, uint32_t airtime_us, uint32_t duty_permille);
