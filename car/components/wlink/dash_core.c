#include "lbr/dash_core.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/dash_link.h"
#include "lbr/lora_policy.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr/wlink.h"

/* Pure dashboard logic; see lbr/dash_core.h. No platform headers here. */

static void lock(lbr_dash_core_t *c)
{
    if (c->hal.lock != NULL) {
        c->hal.lock(c->hal.ctx);
    }
}

static void unlock(lbr_dash_core_t *c)
{
    if (c->hal.unlock != NULL) {
        c->hal.unlock(c->hal.ctx);
    }
}

void lbr_dash_core_init(lbr_dash_core_t *c, const lbr_node_config_t *cfg, const lbr_dash_hal_t *hal)
{
    memset(c, 0, sizeof *c);
    c->cfg = cfg;
    c->hal = *hal;
    lbr_value_table_init(&c->values, LBR_DASH_STALE_US);
    for (size_t i = 0; i < lbr_dash_signal_count; i++) {
        lbr_value_table_watch(&c->values, lbr_dash_signals[i], 0);
    }
    for (size_t i = 0; i < cfg->n_acq && i < LBR_NODE_MAX_SENSORS; i++) {
        lbr_acq_slot_init(&c->slot[i], cfg->acq[i].sensor, cfg->acq[i].rate_hz, (uint16_t)i);
    }
}

typedef struct {
    lbr_dash_core_t *c;
    uint64_t         rx_us;
} rx_ctx_t;

static void put_node(lbr_dash_core_t *c, const uint8_t *lite, uint64_t rx_us)
{
    uint8_t          id   = lite[0];
    lbr_dash_node_t *slot = NULL;
    for (size_t i = 0; i < LBR_DASH_MAX_NODES; i++) {
        if (c->nodes[i].node_id == id) {
            slot = &c->nodes[i];
            break;
        }
        if (c->nodes[i].node_id == 0 && slot == NULL) {
            slot = &c->nodes[i];
        }
    }
    if (slot == NULL || id == 0) {
        atomic_fetch_add(&c->rejected, 1);
        return;
    }
    slot->node_id       = id;
    slot->state         = lite[1];
    slot->flags         = lbr_get_u16(lite + 2);
    slot->sensor_status = lbr_get_u32(lite + 4);
    slot->rx_us         = rx_us;
}

static void on_record(uint8_t type, const uint8_t *payload, size_t len, void *ctx)
{
    rx_ctx_t        *r = ctx;
    lbr_dash_core_t *c = r->c;
    if (type == LBR_REC_SAMPLE) {
        lbr_sample_t s;
        if (lbr_sample_decode(payload, len, &s) == LBR_SAMPLE_WIRE_SIZE) {
            lbr_value_table_offer(&c->values, &s, r->rx_us);
        } else {
            atomic_fetch_add(&c->rejected, 1);
        }
    } else if (type == LBR_REC_HEALTH_LITE && len == LBR_HEALTH_LITE_SIZE) {
        put_node(c, payload, r->rx_us);
    }
}

void lbr_dash_core_on_packet(lbr_dash_core_t *c, const uint8_t *buf, size_t len, uint64_t rx_us)
{
    lbr_pkt_header_t hdr;
    if (!lbr_pkt_parse(buf, len, &hdr, NULL, NULL) || hdr.source != LBR_NODE_CENTRAL ||
        hdr.link != LBR_LINK_DASH) {
        atomic_fetch_add(&c->rejected, 1);
        return;
    }
    lock(c);
    if (c->have_seq) {
        int32_t d = lbr_seq_delta(c->last_seq, hdr.seq);
        if (d > 1) {
            atomic_fetch_add(&c->packet_gaps, (uint32_t)(d - 1));
        }
        if (d > 0) {
            c->last_seq = hdr.seq;
        }
    } else {
        c->have_seq = true;
        c->last_seq = hdr.seq;
    }
    c->last_rx_us = rx_us;
    rx_ctx_t r    = {c, rx_us};
    lbr_pkt_parse(buf, len, &hdr, on_record, &r);
    unlock(c);
    atomic_fetch_add(&c->packets, 1);
}

void lbr_dash_core_slot_start(lbr_dash_core_t *c, size_t i)
{
    lbr_acq_slot_start(&c->slot[i], c->hal.mono_us(c->hal.ctx), NULL, NULL);
}

uint64_t lbr_dash_core_slot_next_due(lbr_dash_core_t *c, size_t i)
{
    return lbr_acq_next_due(c->hal.mono_us(c->hal.ctx), c->slot[i].period_us);
}

void lbr_dash_core_slot_tick(lbr_dash_core_t *c, size_t i)
{
    lbr_acq_slot_t   *a = &c->slot[i];
    lbr_reading_t     readings[LBR_SENSOR_MAX_SIGNALS];
    lbr_acq_tick_t    tick;
    uint64_t          now = c->hal.mono_us(c->hal.ctx);
    uint8_t           pkt[256];
    lbr_pkt_builder_t b;

    if (!lbr_acq_slot_tick(a, now, now, 0, readings, &tick, NULL, NULL)) {
        return;
    }
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_DASH, c->cfg->node_id, c->tx_seq, now);
    for (size_t ch = 0; ch < a->sensor->n_signals; ch++) {
        lbr_sample_t s;
        uint8_t      rec[LBR_SAMPLE_WIRE_SIZE];
        lbr_reading_to_sample(c->cfg->node_id, a->sensor->signals[ch], &readings[ch], tick.k,
                              a->period_us, false, &s);
        if (lbr_sample_encode(&s, rec, sizeof rec) > 0) {
            lbr_pkt_add(&b, LBR_REC_SAMPLE, rec, sizeof rec);
        }
    }
    size_t n = lbr_pkt_finish(&b);
    if (n > 0) {
        c->tx_seq++;
        if (c->hal.send(c->hal.ctx, pkt, n)) {
            atomic_fetch_add(&c->sent, 1);
        } else {
            atomic_fetch_add(&c->send_failed, 1);
        }
    }
}

bool lbr_dash_core_link_up(lbr_dash_core_t *c, uint64_t now_us)
{
    lock(c);
    bool up = c->have_seq &&
              (now_us <= c->last_rx_us || now_us - c->last_rx_us <= LBR_WLINK_LINK_TIMEOUT_US);
    unlock(c);
    return up;
}

void lbr_dash_core_snapshot(lbr_dash_core_t *c, lbr_value_table_t *values,
                            lbr_dash_node_t nodes[LBR_DASH_MAX_NODES])
{
    lock(c);
    *values = c->values;
    memcpy(nodes, c->nodes, sizeof c->nodes);
    unlock(c);
}
