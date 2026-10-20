#include "lbr/lora_policy.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"

const lbr_lora_item_t lbr_lora_default_items[] = {
    {LBR_SIG_ENGINE_RPM, 500},          {LBR_SIG_CVT_RPM, 500},
    {LBR_SIG_GPS_SPEED, 1000},          {LBR_SIG_TEMP_CVT, 1000},
    {LBR_SIG_TEMP_BRAKE_FL, 2000},      {LBR_SIG_TEMP_BRAKE_FR, 2000},
    {LBR_SIG_TEMP_BRAKE_BL, 2000},      {LBR_SIG_TEMP_BRAKE_BR, 2000},
    {LBR_SIG_FUEL_LEVEL, 5000},         {LBR_SIG_GPS_LATITUDE, 2000},
    {LBR_SIG_GPS_LONGITUDE, 2000},      {LBR_SIG_GPS_FIX, 5000},
    {LBR_SIG_CORNER_LINK, 5000},        {LBR_SIG_CORNER_FL_BATTERY, 10000},
    {LBR_SIG_CORNER_FR_BATTERY, 10000}, {LBR_SIG_CORNER_BL_BATTERY, 10000},
    {LBR_SIG_CORNER_BR_BATTERY, 10000}, {LBR_SIG_DASH_BATTERY, 10000},
};
const size_t lbr_lora_default_item_count =
    sizeof lbr_lora_default_items / sizeof lbr_lora_default_items[0];

void lbr_lora_policy_init(lbr_lora_policy_t *p, const lbr_lora_item_t *items, size_t n_items,
                          uint32_t health_interval_ms)
{
    memset(p, 0, sizeof *p);
    if (n_items > LBR_LORA_MAX_ITEMS) {
        n_items = LBR_LORA_MAX_ITEMS;
    }
    for (size_t i = 0; i < n_items; i++) {
        p->slots[i].item = items[i];
    }
    p->n_slots            = n_items;
    p->health_interval_ms = health_interval_ms;
}

static bool is_fault(const lbr_sample_t *s)
{
    return s->signal == LBR_SIG_FAULT;
}

static void queue_event(lbr_lora_policy_t *p, const lbr_sample_t *s)
{
    if (p->evt_count < LBR_LORA_EVT_QUEUE) {
        p->events[(p->evt_head + p->evt_count) % LBR_LORA_EVT_QUEUE] = *s;
        p->evt_count++;
        return;
    }
    p->events_dropped++;
    if (!is_fault(s)) {
        return; /* queue full of older events/faults: drop the new event */
    }
    /* Make room for a fault by evicting the oldest plain event, if any. */
    for (size_t k = 0; k < p->evt_count; k++) {
        size_t idx = (p->evt_head + k) % LBR_LORA_EVT_QUEUE;
        if (!is_fault(&p->events[idx])) {
            for (size_t j = k; j + 1 < p->evt_count; j++) {
                size_t a     = (p->evt_head + j) % LBR_LORA_EVT_QUEUE;
                size_t b     = (p->evt_head + j + 1) % LBR_LORA_EVT_QUEUE;
                p->events[a] = p->events[b];
            }
            p->events[(p->evt_head + p->evt_count - 1) % LBR_LORA_EVT_QUEUE] = *s;
            return;
        }
    }
}

void lbr_lora_policy_offer_sample(lbr_lora_policy_t *p, const lbr_sample_t *s)
{
    if (s->signal == LBR_SIG_FAULT || s->signal == LBR_SIG_EVENT) {
        queue_event(p, s);
        return;
    }
    for (size_t i = 0; i < p->n_slots; i++) {
        if (p->slots[i].item.signal == s->signal) {
            p->slots[i].last  = *s;
            p->slots[i].have  = true;
            p->slots[i].fresh = true;
            return;
        }
    }
}

void lbr_health_lite_encode(const lbr_health_record_t *h, uint8_t out[LBR_HEALTH_LITE_SIZE])
{
    out[0] = h->node_id;
    out[1] = h->state;
    lbr_put_u16(out + 2, h->flags);
    lbr_put_u32(out + 4, h->sensor_status);
}

void lbr_lora_policy_offer_health(lbr_lora_policy_t *p, const lbr_health_record_t *h)
{
    lbr_lora_node_t *free_slot = NULL;
    for (size_t i = 0; i < LBR_LORA_MAX_NODES; i++) {
        lbr_lora_node_t *n = &p->nodes[i];
        if (n->node_id == h->node_id) {
            lbr_health_lite_encode(h, n->lite);
            n->fresh = true;
            return;
        }
        if (n->node_id == 0 && free_slot == NULL) {
            free_slot = n;
        }
    }
    if (free_slot == NULL || h->node_id == 0) {
        p->nodes_untracked++;
        return;
    }
    free_slot->node_id = h->node_id;
    lbr_health_lite_encode(h, free_slot->lite);
    free_slot->fresh        = true;
    free_slot->last_sent_us = 0;
}

static bool due(uint64_t now, uint64_t last_sent, uint32_t interval_ms)
{
    return last_sent == 0 || now - last_sent >= (uint64_t)interval_ms * 1000u;
}

size_t lbr_lora_policy_build(lbr_lora_policy_t *p, uint64_t now_us, uint8_t source, uint32_t seq,
                             uint8_t *buf, size_t cap)
{
    lbr_pkt_builder_t b;
    uint8_t           rec[LBR_SAMPLE_WIRE_SIZE];
    lbr_pkt_begin(&b, buf, cap, LBR_LINK_LORA, source, seq, now_us);

    /* 1. Faults and events. */
    while (p->evt_count > 0) {
        lbr_sample_encode(&p->events[p->evt_head], rec, sizeof rec);
        if (!lbr_pkt_add(&b, LBR_REC_SAMPLE, rec, LBR_SAMPLE_WIRE_SIZE)) {
            break;
        }
        p->evt_head = (p->evt_head + 1) % LBR_LORA_EVT_QUEUE;
        p->evt_count--;
    }

    /* 2. Node health. */
    for (size_t i = 0; i < LBR_LORA_MAX_NODES; i++) {
        lbr_lora_node_t *n = &p->nodes[i];
        if (n->node_id == 0 || !n->fresh || !due(now_us, n->last_sent_us, p->health_interval_ms)) {
            continue;
        }
        if (!lbr_pkt_add(&b, LBR_REC_HEALTH_LITE, n->lite, LBR_HEALTH_LITE_SIZE)) {
            break;
        }
        n->fresh        = false;
        n->last_sent_us = now_us;
    }

    /* 3. Subset signals, round-robin. */
    for (size_t k = 0; k < p->n_slots; k++) {
        size_t           i = (p->rr + k) % p->n_slots;
        lbr_lora_slot_t *s = &p->slots[i];
        if (!s->have || !s->fresh || !due(now_us, s->last_sent_us, s->item.interval_ms)) {
            continue;
        }
        lbr_sample_encode(&s->last, rec, sizeof rec);
        if (!lbr_pkt_add(&b, LBR_REC_SAMPLE, rec, LBR_SAMPLE_WIRE_SIZE)) {
            p->rr = i; /* start here next time */
            break;
        }
        s->fresh        = false;
        s->last_sent_us = now_us;
    }
    return lbr_pkt_finish(&b);
}

uint32_t lbr_lora_airtime_us(const lbr_lora_modem_t *m, size_t payload_len)
{
    uint64_t tsym_us = ((uint64_t)1u << m->sf) * 1000000u / m->bw_hz;
    int      de      = tsym_us > 16000u ? 1 : 0;
    int64_t  num =
        8 * (int64_t)payload_len - 4 * m->sf + 28 + (m->crc ? 16 : 0) - (m->implicit ? 20 : 0);
    int64_t den       = 4 * ((int64_t)m->sf - 2 * de);
    int64_t blocks    = num > 0 ? (num + den - 1) / den : 0;
    int64_t n_payload = 8 + blocks * (m->cr + 4);
    /* Preamble is (n + 4.25) symbols: computed in quarter symbols. */
    uint64_t preamble_q = ((uint64_t)m->preamble * 4u + 17u) * tsym_us / 4u;
    return (uint32_t)(preamble_q + (uint64_t)n_payload * tsym_us);
}

uint64_t lbr_lora_next_tx_us(uint64_t now_us, uint32_t airtime_us, uint32_t duty_permille)
{
    if (duty_permille == 0 || duty_permille >= 1000u) {
        return now_us;
    }
    return now_us + (uint64_t)airtime_us * (1000u - duty_permille) / duty_permille;
}
