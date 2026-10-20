#include "lbr/wlink.h"

#include <string.h>

#include "lbr/bytes.h"

/* Pure wireless-link codecs; see lbr/wlink.h. */

size_t lbr_imu_batch_encode(const lbr_imu_batch_t *b, uint8_t *buf, size_t cap)
{
    size_t len = LBR_WLINK_BATCH_HDR + (size_t)b->n * b->sample_size;
    if (len > cap || b->n > LBR_WLINK_MAX_BATCH || b->sample_size > LBR_WLINK_MAX_SAMPLE) {
        return 0;
    }
    lbr_put_u32(buf, b->k0);
    buf[4] = b->n;
    buf[5] = b->battery_raw;
    buf[6] = b->flags;
    buf[7] = b->sample_size;
    memcpy(buf + LBR_WLINK_BATCH_HDR, b->samples, (size_t)b->n * b->sample_size);
    return len;
}

bool lbr_imu_batch_decode(const uint8_t *buf, size_t len, lbr_imu_batch_t *b)
{
    if (len < LBR_WLINK_BATCH_HDR) {
        return false;
    }
    b->k0          = lbr_get_u32(buf);
    b->n           = buf[4];
    b->battery_raw = buf[5];
    b->flags       = buf[6];
    b->sample_size = buf[7];
    b->samples     = buf + LBR_WLINK_BATCH_HDR;
    return b->n <= LBR_WLINK_MAX_BATCH && b->sample_size <= LBR_WLINK_MAX_SAMPLE &&
           len == LBR_WLINK_BATCH_HDR + (size_t)b->n * b->sample_size;
}

static size_t payload_bytes(const lbr_frame_def_t *d)
{
    return d->dlc - (d->counter_offset != LBR_FRAME_NO_COUNTER ? 2u : 0u);
}

size_t lbr_wlink_origin_frames(uint8_t origin, const lbr_frame_def_t **frames, size_t cap,
                               size_t *sample_size)
{
    size_t n    = 0;
    size_t size = 0;
    for (size_t i = 0; i < lbr_frame_count() && n < cap; i++) {
        const lbr_frame_def_t *d = lbr_frame_at(i);
        if (d->origin == origin && d->sender != origin) {
            frames[n++] = d;
            size += payload_bytes(d);
        }
    }
    if (sample_size != NULL) {
        *sample_size = size;
    }
    return n;
}

size_t lbr_wlink_strip_counter(const lbr_frame_def_t *d, const lbr_can_frame_t *f, uint8_t *out)
{
    size_t n = 0;
    for (size_t i = 0; i < d->dlc; i++) {
        bool counter = d->counter_offset != LBR_FRAME_NO_COUNTER &&
                       (i == d->counter_offset || i == d->counter_offset + 1u);
        if (!counter) {
            out[n++] = f->data[i];
        }
    }
    return n;
}

void lbr_wlink_build_frame(const lbr_frame_def_t *d, const uint8_t *bytes, uint64_t k,
                           lbr_can_frame_t *out)
{
    size_t n = 0;
    out->id  = d->can_id;
    out->dlc = d->dlc;
    memset(out->data, 0, sizeof out->data);
    for (size_t i = 0; i < d->dlc; i++) {
        bool counter = d->counter_offset != LBR_FRAME_NO_COUNTER &&
                       (i == d->counter_offset || i == d->counter_offset + 1u);
        if (!counter) {
            out->data[i] = bytes[n++];
        }
    }
    if (d->counter_offset != LBR_FRAME_NO_COUNTER) {
        lbr_put_u16(out->data + d->counter_offset, (uint16_t)k);
    }
}

/* ------------------------------------------------------------------------ */
/* Clock                                                                     */

void lbr_wclock_request(lbr_wclock_t *c, uint64_t local_now, uint8_t out[LBR_WLINK_TIME_REQ])
{
    c->pending = true;
    c->t1      = local_now;
    lbr_put_u64(out, local_now);
}

bool lbr_wclock_on_response(lbr_wclock_t *c, const uint8_t *payload, size_t len, uint64_t local_rx,
                            uint64_t *shared_at_rx, uint16_t *epoch)
{
    if (len != LBR_WLINK_TIME_RESP) {
        c->rejected++;
        return false;
    }
    uint64_t t1 = lbr_get_u64(payload);
    uint64_t t2 = lbr_get_u64(payload + 8);
    uint64_t t3 = lbr_get_u64(payload + 16);
    if (!c->pending || t1 != c->t1 || local_rx < t1 || t3 < t2) {
        c->rejected++; /* not the answer to our outstanding request */
        return false;
    }
    c->pending     = false;
    uint64_t total = local_rx - t1;
    uint64_t held  = t3 - t2;
    uint64_t rtt   = total > held ? total - held : 0u;
    c->last_rtt_us = rtt > UINT32_MAX ? UINT32_MAX : (uint32_t)rtt;
    if (rtt > LBR_WLINK_MAX_RTT_US) {
        c->rejected++; /* a slow round trip says little about the offset */
        return false;
    }
    c->accepted++;
    c->server_synced = (payload[26] & 0x01u) != 0;
    *shared_at_rx    = t3 + rtt / 2u;
    *epoch           = lbr_get_u16(payload + 24);
    return true;
}

size_t lbr_wclock_reply(const uint8_t *req, size_t len, uint64_t t2_shared, uint64_t t3_shared,
                        uint16_t epoch, bool server_synced, uint8_t out[LBR_WLINK_TIME_RESP])
{
    if (len != LBR_WLINK_TIME_REQ) {
        return 0;
    }
    memcpy(out, req, 8);
    lbr_put_u64(out + 8, t2_shared);
    lbr_put_u64(out + 16, t3_shared);
    lbr_put_u16(out + 24, epoch);
    out[26] = server_synced ? 0x01u : 0u;
    return LBR_WLINK_TIME_RESP;
}
