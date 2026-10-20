#include "lbr/corner_rx.h"

#include <string.h>

#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"

/* Pure IMU Controller corner receiver; see lbr/corner_rx.h. */

#define WINDOW 64u

static void lock(lbr_corner_rx_t *rx)
{
    if (rx->hal.lock != NULL) {
        rx->hal.lock(rx->hal.ctx);
    }
}

static void unlock(lbr_corner_rx_t *rx)
{
    if (rx->hal.unlock != NULL) {
        rx->hal.unlock(rx->hal.ctx);
    }
}

void lbr_corner_rx_init(lbr_corner_rx_t *rx, const lbr_corner_rx_hal_t *hal)
{
    memset(rx, 0, sizeof *rx);
    rx->hal = *hal;
    for (unsigned i = 0; i < LBR_CORNER_COUNT; i++) {
        lbr_corner_peer_t *p = &rx->peer[i];
        p->n_frames = lbr_wlink_origin_frames((uint8_t)(LBR_NODE_CORNER_FL + i), p->frames, 4,
                                              &p->sample_size);
    }
}

static unsigned zeros(uint64_t bits, unsigned n)
{
    unsigned z = 0;
    for (unsigned i = 0; i < n; i++) {
        z += ((bits >> i) & 1u) ? 0u : 1u;
    }
    return z;
}

static void forward(lbr_corner_rx_t *rx, lbr_corner_peer_t *p, uint64_t k, const uint8_t *bytes)
{
    for (size_t f = 0; f < p->n_frames; f++) {
        lbr_can_frame_t frame;
        lbr_wlink_build_frame(p->frames[f], bytes, k, &frame);
        size_t used =
            p->frames[f]->dlc - (p->frames[f]->counter_offset != LBR_FRAME_NO_COUNTER ? 2u : 0u);
        bytes += used;
        if (!rx->hal.send_frame(rx->hal.ctx, &frame)) {
            p->tx_failed++;
        }
    }
    p->forwarded++;
}

/* Place sample k; forward it if it is new. */
static void take(lbr_corner_rx_t *rx, lbr_corner_peer_t *p, uint64_t k, const uint8_t *bytes)
{
    if (!p->have_k) {
        /* Nothing before the first sample can be missing. */
        p->have_k = true;
        p->next_k = k;
        p->window = ~0ull;
    }
    if (k >= p->next_k) {
        uint64_t shift = k + 1u - p->next_k;
        if (shift >= WINDOW) {
            p->lost += zeros(p->window, WINDOW) + (uint32_t)(shift - WINDOW);
            p->window = 0;
        } else {
            p->lost += zeros(p->window >> (WINDOW - shift), (unsigned)shift);
            p->window <<= shift;
        }
        p->next_k = k + 1u;
        p->window |= 1u;
        forward(rx, p, k, bytes);
        return;
    }
    uint64_t i = p->next_k - 1u - k;
    if (i >= WINDOW) {
        p->late++;
    } else if (p->window & (1ull << i)) {
        p->duplicates++;
    } else {
        p->window |= 1ull << i;
        forward(rx, p, k, bytes);
    }
}

typedef struct {
    lbr_corner_rx_t   *rx;
    lbr_corner_peer_t *p;
    uint8_t            node;
    uint64_t           rx_us;
} rx_ctx_t;

static void on_record(uint8_t type, const uint8_t *payload, size_t len, void *ctx)
{
    rx_ctx_t        *r  = ctx;
    lbr_corner_rx_t *rx = r->rx;
    if (type == LBR_REC_TIME_REQ) {
        uint8_t resp[LBR_WLINK_TIME_RESP];
        size_t  n = lbr_wclock_reply(payload, len, r->rx_us, rx->hal.now_us(rx->hal.ctx),
                                     rx->hal.epoch(rx->hal.ctx), rx->hal.synced(rx->hal.ctx), resp);
        if (n == 0) {
            rx->rejected++;
            return;
        }
        uint8_t           pkt[64];
        lbr_pkt_builder_t b;
        lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_CORNER, LBR_NODE_IMU, rx->reply_seq++,
                      rx->hal.now_us(rx->hal.ctx));
        lbr_pkt_add(&b, LBR_REC_TIME_RESP, resp, n);
        size_t plen = lbr_pkt_finish(&b);
        rx->hal.reply(rx->hal.ctx, r->node, pkt, plen);
        return;
    }
    if (type != LBR_REC_IMU_BATCH) {
        return; /* unknown record types are skipped, by the compatibility rules */
    }
    lbr_corner_peer_t *p = r->p;
    lbr_imu_batch_t    b;
    if (!lbr_imu_batch_decode(payload, len, &b) || b.sample_size != p->sample_size) {
        rx->rejected++;
        return;
    }
    if (p->have_k && ((p->flags ^ b.flags) & LBR_WLINK_F_SYNCED)) {
        /* The corner's clock joined (or left) the shared timeline: its grid
         * indexes jumped. A new timeline, not lost samples. */
        p->have_k = false;
    }
    p->battery_raw = b.battery_raw;
    p->flags       = b.flags;
    for (uint8_t i = 0; i < b.n; i++) {
        /* Extend the 32-bit index against the newest sample forwarded. */
        uint32_t k32 = b.k0 + i;
        uint64_t k   = k32;
        if (p->have_k) {
            int64_t kk = (int64_t)p->next_k + (int32_t)(k32 - (uint32_t)p->next_k);
            k          = kk < 0 ? 0u : (uint64_t)kk;
        }
        take(rx, p, k, b.samples + (size_t)i * b.sample_size);
    }
}

void lbr_corner_rx_on_packet(lbr_corner_rx_t *rx, const uint8_t *buf, size_t len,
                             uint64_t rx_shared_us)
{
    lbr_pkt_header_t hdr;
    lock(rx);
    int corner = -1;
    if (lbr_pkt_parse(buf, len, &hdr, NULL, NULL)) {
        corner = lbr_node_corner_index(hdr.source);
    }
    if (corner < 0 || hdr.link != LBR_LINK_CORNER) {
        rx->rejected++;
        unlock(rx);
        return;
    }
    lbr_corner_peer_t *p = &rx->peer[corner];
    if (p->have_seq) {
        int32_t d = lbr_seq_delta(p->last_seq, hdr.seq);
        if (d > 1) {
            p->packet_gaps += (uint32_t)(d - 1);
        }
        if (d > 0) {
            p->last_seq = hdr.seq;
        }
    } else {
        p->have_seq = true;
        p->last_seq = hdr.seq;
    }
    p->packets++;
    p->last_rx_us = rx_shared_us;
    bool found    = !p->up;
    p->up         = true;
    p->seen       = true;

    rx_ctx_t r = {rx, p, hdr.source, rx_shared_us};
    lbr_pkt_parse(buf, len, &hdr, on_record, &r);
    unlock(rx);
    if (found && rx->hal.event != NULL) {
        rx->hal.event(rx->hal.ctx, LBR_EVT_NODE_FOUND, hdr.source, false);
    }
}

uint8_t lbr_corner_rx_tick(lbr_corner_rx_t *rx, uint32_t *new_loss)
{
    uint8_t lost_nodes[LBR_CORNER_COUNT];
    size_t  n_lost = 0;
    uint8_t flags  = 0;

    lock(rx);
    uint64_t now  = rx->hal.now_us(rx->hal.ctx);
    uint32_t loss = 0;
    for (unsigned i = 0; i < LBR_CORNER_COUNT; i++) {
        lbr_corner_peer_t *p = &rx->peer[i];
        if (p->up && now > p->last_rx_us && now - p->last_rx_us > LBR_WLINK_LINK_TIMEOUT_US) {
            p->up                = false;
            lost_nodes[n_lost++] = (uint8_t)(LBR_NODE_CORNER_FL + i);
        }
        if (!p->up) {
            flags |= LBR_HB_ERR_LINK;
        } else if (!(p->flags & LBR_WLINK_F_SENSOR_OK)) {
            flags |= LBR_HB_ERR_SENSOR;
        }
        loss += p->lost + p->tx_failed;
    }
    if (new_loss != NULL) {
        *new_loss += loss - rx->reported_loss;
    }
    rx->reported_loss = loss;
    unlock(rx);

    for (size_t i = 0; i < n_lost; i++) {
        if (rx->hal.event != NULL) {
            rx->hal.event(rx->hal.ctx, LBR_EVT_NODE_LOST, lost_nodes[i], true);
        }
    }
    return flags;
}

void lbr_corner_rx_service(lbr_corner_rx_t *rx, lbr_node_core_t *core)
{
    uint32_t loss  = 0;
    uint8_t  flags = lbr_corner_rx_tick(rx, &loss);
    lbr_node_core_set_app_flags(core, flags);
    if (loss > 0) {
        lbr_node_core_count_loss(core, loss);
    }
}

uint8_t lbr_corner_rx_link_mask(lbr_corner_rx_t *rx)
{
    uint8_t m = 0;
    lock(rx);
    for (unsigned i = 0; i < LBR_CORNER_COUNT; i++) {
        m |= rx->peer[i].up ? (uint8_t)(1u << i) : 0u;
    }
    unlock(rx);
    return m;
}

uint8_t lbr_corner_rx_synced_mask(lbr_corner_rx_t *rx)
{
    uint8_t m = 0;
    lock(rx);
    for (unsigned i = 0; i < LBR_CORNER_COUNT; i++) {
        const lbr_corner_peer_t *p = &rx->peer[i];
        m |= (p->up && (p->flags & LBR_WLINK_F_SYNCED)) ? (uint8_t)(1u << i) : 0u;
    }
    unlock(rx);
    return m;
}

/* ------------------------------------------------------------------------ */
/* CORNER_STATUS driver                                                      */

static int status_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}

static int status_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us;
    lbr_corner_rx_t *rx     = self->ctx;
    uint8_t          link   = lbr_corner_rx_link_mask(rx);
    uint8_t          synced = lbr_corner_rx_synced_mask(rx);
    for (size_t ch = 0; ch < self->n_signals; ch++) {
        lbr_signal_id_t sig = self->signals[ch];
        lbr_reading_t  *r   = &out[ch];
        memset(r, 0, sizeof *r);
        if (sig == LBR_SIG_CORNER_LINK) {
            r->valid = true;
            r->v.u32 = link;
        } else if (sig == LBR_SIG_CORNER_SYNCED) {
            r->valid = true;
            r->v.u32 = synced;
        } else if (sig >= LBR_SIG_CORNER_FL_BATTERY && sig <= LBR_SIG_CORNER_BR_BATTERY) {
            unsigned               i = (unsigned)(sig - LBR_SIG_CORNER_FL_BATTERY);
            size_t                 fi;
            const lbr_frame_def_t *d      = lbr_frame_for_signal(sig, &fi);
            lbr_sample_t           s      = {0};
            uint8_t                raw[8] = {0};
            lock(rx);
            raw[0]  = rx->peer[i].battery_raw;
            bool up = rx->peer[i].up;
            unlock(rx);
            if (!up || d == NULL) {
                r->reason = LBR_UNAVAIL_NO_DATA;
                continue;
            }
            /* The corner sent its battery already in the field encoding. */
            lbr_field_def_t fd = d->fields[fi];
            fd.offset          = 0;
            lbr_can_field_decode(&fd, raw, &s);
            r->valid  = s.valid;
            r->reason = s.valid ? LBR_UNAVAIL_UNSPECIFIED : (lbr_unavail_reason_t)s.v.u32;
            r->v.f32  = s.valid ? s.v.f32 : 0.0f;
        } else {
            r->reason = LBR_UNAVAIL_NOT_FITTED;
        }
    }
    return 0;
}

void lbr_sensor_corner_status(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                              size_t n_signals, lbr_corner_rx_t *rx)
{
    *out = (lbr_sensor_t){
        .name      = name,
        .signals   = signals,
        .n_signals = n_signals,
        .init      = status_init,
        .read      = status_read,
        .ctx       = rx,
    };
}
