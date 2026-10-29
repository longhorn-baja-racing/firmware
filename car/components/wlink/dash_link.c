#include "lbr/dash_link.h"

#include <string.h>

#include "lbr/lora_policy.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr/wlink.h"

/* Pure Central-side dashboard link; see lbr/dash_link.h. */

const lbr_signal_id_t lbr_dash_signals[] = {
    LBR_SIG_ENGINE_RPM,    LBR_SIG_CVT_RPM,       LBR_SIG_GPS_SPEED,     LBR_SIG_FUEL_LEVEL,
    LBR_SIG_TEMP_CVT,      LBR_SIG_TEMP_BRAKE_FL, LBR_SIG_TEMP_BRAKE_FR, LBR_SIG_TEMP_BRAKE_BL,
    LBR_SIG_TEMP_BRAKE_BR, LBR_SIG_CORNER_LINK,   LBR_SIG_BRAKE_PEDAL,   LBR_SIG_GAS_PEDAL,
    LBR_SIG_FAULT,
};
const size_t lbr_dash_signal_count = sizeof lbr_dash_signals / sizeof lbr_dash_signals[0];

void lbr_dash_link_init(lbr_dash_link_t *d)
{
    memset(d, 0, sizeof *d);
}

static void on_record(uint8_t type, const uint8_t *payload, size_t len, void *ctx)
{
    lbr_dash_link_t *d = ctx;
    lbr_sample_t     s;
    if (type != LBR_REC_SAMPLE) {
        return; /* unknown record types are skipped */
    }
    if (lbr_sample_decode(payload, len, &s) != LBR_SAMPLE_WIRE_SIZE) {
        d->rejected++;
        return;
    }
    if (s.signal == LBR_SIG_DASH_BATTERY) {
        d->battery      = s;
        d->have_battery = true;
    }
}

void lbr_dash_link_on_packet(lbr_dash_link_t *d, const uint8_t *buf, size_t len, uint64_t rx_us)
{
    lbr_pkt_header_t hdr;
    if (!lbr_pkt_parse(buf, len, &hdr, NULL, NULL) || hdr.source != LBR_NODE_DASHBOARD ||
        hdr.link != LBR_LINK_DASH) {
        d->rejected++;
        return;
    }
    if (d->seq.have && rx_us > d->last_rx_us && rx_us - d->last_rx_us > LBR_WLINK_DASH_TIMEOUT_US) {
        d->seq.have = false; /* back after an outage, maybe rebooted: seq restarts */
    }
    lbr_seq_result_t sr = lbr_seq_window_take(&d->seq, hdr.seq, &d->packet_gaps);
    if (sr == LBR_SEQ_OLD) {
        d->seq.have = false; /* the dashboard restarted its count */
        sr          = lbr_seq_window_take(&d->seq, hdr.seq, &d->packet_gaps);
    }
    if (sr != LBR_SEQ_NEW) {
        d->rejected += sr == LBR_SEQ_DUPLICATE ? 1u : 0u; /* older than what we have */
        return;
    }
    d->packets++;
    d->seen       = true;
    d->last_rx_us = rx_us;
    lbr_pkt_parse(buf, len, &hdr, on_record, d);
}

int lbr_dash_link_tick(lbr_dash_link_t *d, uint64_t now_us)
{
    bool up =
        d->seen && (now_us <= d->last_rx_us || now_us - d->last_rx_us <= LBR_WLINK_DASH_TIMEOUT_US);
    int change = up == d->up ? 0 : (up ? 1 : -1);
    d->up      = up;
    return change;
}

size_t lbr_dash_link_build(lbr_dash_link_t *d, uint64_t now_us, uint8_t source,
                           const lbr_value_table_t *values, const lbr_node_table_t *nodes,
                           uint8_t *buf, size_t cap)
{
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, buf, cap, LBR_LINK_DASH, source, d->tx_seq, now_us);
    for (size_t i = 0; i < lbr_dash_signal_count; i++) {
        lbr_sample_t      s;
        lbr_value_state_t st = lbr_value_table_get(values, lbr_dash_signals[i], now_us, &s);
        if (st == LBR_VALUE_NONE) {
            continue;
        }
        if (st == LBR_VALUE_STALE) {
            lbr_sample_set_unavailable(&s, LBR_UNAVAIL_NO_DATA);
        }
        uint8_t rec[LBR_SAMPLE_WIRE_SIZE];
        if (lbr_sample_encode(&s, rec, sizeof rec) > 0) {
            lbr_pkt_add(&b, LBR_REC_SAMPLE, rec, sizeof rec);
        }
    }
    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE; i++) {
        const lbr_node_entry_t *e = &nodes->nodes[i];
        if (e->node_id == 0) {
            continue;
        }
        lbr_health_record_t h = e->health;
        uint8_t             lite[LBR_HEALTH_LITE_SIZE];
        h.node_id = e->node_id;
        h.state   = !e->present ? (uint8_t)LBR_NODE_FAULT
                                : (e->hb_seen ? e->hb_state : (uint8_t)LBR_NODE_BOOT);
        lbr_health_lite_encode(&h, lite);
        lbr_pkt_add(&b, LBR_REC_HEALTH_LITE, lite, sizeof lite);
    }
    size_t n = lbr_pkt_finish(&b);
    if (n > 0) {
        d->tx_seq++;
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* DASH_STATUS driver                                                        */

static int dash_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}

static int dash_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us;
    lbr_dash_link_t *d = self->ctx;
    if (d->lock != NULL) {
        d->lock(d->lock_ctx);
    }
    bool         up   = d->up;
    bool         have = d->have_battery;
    lbr_sample_t batt = d->battery;
    if (d->unlock != NULL) {
        d->unlock(d->lock_ctx);
    }
    for (size_t ch = 0; ch < self->n_signals; ch++) {
        lbr_reading_t *r = &out[ch];
        memset(r, 0, sizeof *r);
        if (self->signals[ch] == LBR_SIG_DASH_LINK) {
            r->valid = true;
            r->v.b   = up;
        } else if (self->signals[ch] == LBR_SIG_DASH_BATTERY) {
            if (!up || !have) {
                r->reason = LBR_UNAVAIL_NO_DATA;
            } else if (!batt.valid) {
                r->reason = (lbr_unavail_reason_t)batt.v.u32;
            } else {
                r->valid = true;
                r->v.f32 = batt.v.f32;
            }
        } else {
            r->reason = LBR_UNAVAIL_NOT_FITTED;
        }
    }
    return 0;
}

void lbr_sensor_dash_link(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                          size_t n_signals, lbr_dash_link_t *d, void (*lock)(void *),
                          void (*unlock)(void *), void      *lock_ctx)
{
    d->lock     = lock;
    d->unlock   = unlock;
    d->lock_ctx = lock_ctx;
    *out        = (lbr_sensor_t){
        .name      = name,
        .signals   = signals,
        .n_signals = n_signals,
        .init      = dash_init,
        .read      = dash_read,
        .ctx       = d,
    };
}
