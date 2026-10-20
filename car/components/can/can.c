#include "lbr/can.h"

#include <math.h>
#include <string.h>

#include "lbr/bytes.h"
#include "lbr/nodes.h"

#define T_LO_MASK  0x00FFFFFFu
#define T_LO_RANGE 0x01000000

#define EVT_FAULT    0x01u
#define EVT_SYNCED   0x02u
#define EVT_SEQ_MASK 0x3Fu

uint8_t lbr_can_sender(uint32_t id)
{
    if (id > LBR_CAN_ID_MAX) {
        return 0;
    }
    if (id == LBR_CAN_ID_TIME_SYNC || id == LBR_CAN_ID_TIME_FOLLOW_UP || id == LBR_CAN_ID_COMMAND) {
        return LBR_NODE_CENTRAL;
    }
    if (id > LBR_CAN_ID_EVENT_BASE && id <= LBR_CAN_ID_EVENT_BASE + LBR_NODE_CAN_MAX) {
        return (uint8_t)(id - LBR_CAN_ID_EVENT_BASE);
    }
    if (id > LBR_CAN_ID_HEARTBEAT_BASE && id <= LBR_CAN_ID_HEARTBEAT_BASE + LBR_NODE_CAN_MAX) {
        return (uint8_t)(id - LBR_CAN_ID_HEARTBEAT_BASE);
    }
    if (id >= LBR_CAN_ID_HEALTH_BASE) {
        uint32_t node = (id - LBR_CAN_ID_HEALTH_BASE) >> 4;
        uint32_t page = id & 0xFu;
        return (lbr_node_is_can((uint8_t)node) && page < LBR_HEALTH_PAGE_COUNT) ? (uint8_t)node
                                                                                : 0u;
    }
    const lbr_frame_def_t *d = lbr_frame_def((uint16_t)id);
    return d != NULL ? d->sender : 0u;
}

/* ------------------------------------------------------------------------ */
/* Field codec                                                               */

typedef struct {
    int64_t lo, hi;     /* valid raw range */
    int64_t unavail;    /* raw value of reason 0 */
    bool    unavail_lo; /* reserved values sit at the bottom (signed) */
} enc_range_t;

static enc_range_t enc_range(lbr_field_enc_t enc)
{
    switch (enc) {
        case LBR_ENC_I16:
            return (enc_range_t){-32768 + 8, 32767, -32768, true};
        case LBR_ENC_U16:
            return (enc_range_t){0, 0xFFFF - 8, 0xFFF8, false};
        case LBR_ENC_U8:
            return (enc_range_t){0, 0xFF - 8, 0xF8, false};
        case LBR_ENC_I32:
            return (enc_range_t){(int64_t)INT32_MIN + 8, INT32_MAX, INT32_MIN, true};
        case LBR_ENC_U32:
        default:
            return (enc_range_t){0, 0xFFFFFFFFll - 8, 0xFFFFFFF8ll, false};
    }
}

static void put_raw(lbr_field_enc_t enc, uint8_t *p, int64_t raw)
{
    switch (enc) {
        case LBR_ENC_I16:
        case LBR_ENC_U16:
            lbr_put_u16(p, (uint16_t)raw);
            break;
        case LBR_ENC_U8:
            p[0] = (uint8_t)raw;
            break;
        case LBR_ENC_I32:
        case LBR_ENC_U32:
        default:
            lbr_put_u32(p, (uint32_t)raw);
            break;
    }
}

static int64_t get_raw(lbr_field_enc_t enc, const uint8_t *p)
{
    switch (enc) {
        case LBR_ENC_I16:
            return (int16_t)lbr_get_u16(p);
        case LBR_ENC_U16:
            return lbr_get_u16(p);
        case LBR_ENC_U8:
            return p[0];
        case LBR_ENC_I32:
            return (int32_t)lbr_get_u32(p);
        case LBR_ENC_U32:
        default:
            return lbr_get_u32(p);
    }
}

static bool to_raw(const lbr_field_def_t *fd, const lbr_sample_t *s, int64_t *raw)
{
    enc_range_t r = enc_range((lbr_field_enc_t)fd->enc);
    double      v;
    switch (s->type) {
        case LBR_VAL_F32:
            if (!isfinite(s->v.f32)) {
                return false;
            }
            v = round((double)s->v.f32 / (double)fd->scale);
            if (v < (double)r.lo || v > (double)r.hi) {
                return false;
            }
            *raw = (int64_t)v;
            return true;
        case LBR_VAL_I32:
            *raw = s->v.i32;
            break;
        case LBR_VAL_BOOL:
            *raw = s->v.b ? 1 : 0;
            break;
        default:
            *raw = s->v.u32;
            break;
    }
    return *raw >= r.lo && *raw <= r.hi;
}

bool lbr_can_field_encode(const lbr_field_def_t *fd, const lbr_sample_t *s, uint8_t *payload)
{
    enc_range_t r   = enc_range((lbr_field_enc_t)fd->enc);
    uint8_t    *p   = payload + fd->offset;
    int64_t     raw = 0;
    if (s->valid && to_raw(fd, s, &raw)) {
        put_raw((lbr_field_enc_t)fd->enc, p, raw);
        return true;
    }
    uint32_t why = s->valid ? (uint32_t)LBR_UNAVAIL_OUT_OF_RANGE : s->v.u32;
    if (why >= LBR_ENC_UNAVAIL_SLOTS) {
        why = LBR_UNAVAIL_UNSPECIFIED;
    }
    put_raw((lbr_field_enc_t)fd->enc, p, r.unavail + (int64_t)why);
    return !s->valid;
}

void lbr_can_field_decode(const lbr_field_def_t *fd, const uint8_t *payload, lbr_sample_t *s)
{
    const lbr_signal_info_t *info = lbr_signal_info(fd->signal);
    enc_range_t              r    = enc_range((lbr_field_enc_t)fd->enc);
    int64_t                  raw  = get_raw((lbr_field_enc_t)fd->enc, payload + fd->offset);

    s->signal = fd->signal;
    s->type   = info != NULL ? info->type : LBR_VAL_RAW;
    if (raw >= r.unavail && raw < r.unavail + (int64_t)LBR_ENC_UNAVAIL_SLOTS) {
        lbr_sample_set_unavailable(s, (lbr_unavail_reason_t)(raw - r.unavail));
        return;
    }
    s->valid = true;
    switch (s->type) {
        case LBR_VAL_F32:
            s->v.f32 = (float)((double)raw * (double)fd->scale);
            break;
        case LBR_VAL_I32:
            s->v.i32 = (int32_t)raw;
            break;
        case LBR_VAL_BOOL:
            s->v.b = raw != 0;
            break;
        default:
            s->v.u32 = (uint32_t)raw;
            break;
    }
}

unsigned lbr_can_encode_data(const lbr_frame_def_t *d, const lbr_sample_t *values, uint64_t k,
                             lbr_can_frame_t *out)
{
    unsigned misfit = 0;
    out->id         = d->can_id;
    out->dlc        = d->dlc;
    memset(out->data, 0, sizeof out->data);
    for (size_t i = 0; i < d->n_fields; i++) {
        if (!lbr_can_field_encode(&d->fields[i], &values[i], out->data)) {
            misfit++;
        }
    }
    if (d->counter_offset != LBR_FRAME_NO_COUNTER) {
        lbr_put_u16(out->data + d->counter_offset, (uint16_t)k);
    }
    return misfit;
}

uint64_t lbr_can_grid_index(uint16_t counter, uint64_t rx_shared_us, uint32_t period_us)
{
    uint64_t k_rx = rx_shared_us / period_us;
    int32_t  d    = (int16_t)(uint16_t)((uint16_t)k_rx - counter); /* signed 16-bit distance */
    int64_t  k    = (int64_t)k_rx - d;
    return k < 0 ? (uint64_t)counter : (uint64_t)k;
}

size_t lbr_can_decode_data(const lbr_can_frame_t *f, uint64_t rx_shared_us, bool synced,
                           lbr_sample_t *out, size_t cap)
{
    const lbr_frame_def_t *d = lbr_frame_def((uint16_t)f->id);
    if (d == NULL || f->dlc != d->dlc) {
        return 0;
    }
    uint32_t period = lbr_frame_period_us(d);
    bool     infer  = d->counter_offset == LBR_FRAME_NO_COUNTER;
    uint64_t k =
        infer ? rx_shared_us / period
              : lbr_can_grid_index(lbr_get_u16(f->data + d->counter_offset), rx_shared_us, period);
    size_t n = 0;
    for (size_t i = 0; i < d->n_fields && n < cap; i++) {
        lbr_sample_t *s = &out[n++];
        memset(s, 0, sizeof *s);
        lbr_can_field_decode(&d->fields[i], f->data, s);
        s->source       = d->origin;
        s->synced       = synced;
        s->seq_inferred = infer;
        s->seq          = (uint32_t)k;
        s->t_mono_us    = k * period;
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* Events                                                                    */

uint64_t lbr_can_unwrap_time(uint32_t t_lo24, uint64_t ref_us)
{
    /* Signed 24-bit distance from the sample time back to the reference. */
    int32_t d = (int32_t)(((uint32_t)ref_us - t_lo24) & T_LO_MASK);
    if (d >= T_LO_RANGE / 2) {
        d -= T_LO_RANGE;
    }
    int64_t t = (int64_t)ref_us - d;
    return t < 0 ? 0u : (uint64_t)t;
}

bool lbr_can_encode_event(const lbr_sample_t *s, lbr_can_frame_t *out)
{
    if (!lbr_node_is_can(s->source) || (s->signal != LBR_SIG_EVENT && s->signal != LBR_SIG_FAULT)) {
        return false;
    }
    out->id  = LBR_CAN_ID_EVENT_BASE + s->source;
    out->dlc = 8;
    lbr_put_u32(out->data, s->v.u32);
    out->data[4] = (uint8_t)((s->signal == LBR_SIG_FAULT ? EVT_FAULT : 0u) |
                             (s->synced ? EVT_SYNCED : 0u) | ((s->seq & EVT_SEQ_MASK) << 2));
    lbr_put_u24(out->data + 5, (uint32_t)(s->t_mono_us & T_LO_MASK));
    return true;
}

bool lbr_can_decode_event(const lbr_can_frame_t *f, uint64_t rx_shared_us, lbr_sample_t *out)
{
    if (f->id <= LBR_CAN_ID_EVENT_BASE || f->id > LBR_CAN_ID_EVENT_BASE + LBR_NODE_CAN_MAX ||
        f->dlc != 8) {
        return false;
    }
    uint8_t      fl = f->data[4];
    lbr_sample_t s  = {
        .signal    = (fl & EVT_FAULT) ? LBR_SIG_FAULT : LBR_SIG_EVENT,
        .source    = (uint8_t)(f->id - LBR_CAN_ID_EVENT_BASE),
        .type      = LBR_VAL_STATUS,
        .valid     = true,
        .synced    = (fl & EVT_SYNCED) != 0,
        .seq       = (uint32_t)(fl >> 2) & EVT_SEQ_MASK,
        .t_mono_us = lbr_can_unwrap_time(lbr_get_u24(f->data + 5), rx_shared_us),
    };
    s.v.u32 = lbr_get_u32(f->data);
    *out    = s;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Time sync and command                                                     */

void lbr_can_encode_time_sync(uint8_t sync_seq, uint16_t epoch, uint64_t now_us,
                              lbr_can_frame_t *out)
{
    out->id  = LBR_CAN_ID_TIME_SYNC;
    out->dlc = 8;
    memset(out->data, 0, sizeof out->data);
    lbr_put_u32(out->data, (uint32_t)(now_us / 1000u));
    out->data[4] = sync_seq;
    lbr_put_u16(out->data + 5, epoch);
}

void lbr_can_encode_time_follow_up(uint8_t sync_seq, uint64_t tx_us, lbr_can_frame_t *out)
{
    out->id  = LBR_CAN_ID_TIME_FOLLOW_UP;
    out->dlc = 8;
    lbr_put_u64(out->data, tx_us << 8);
    out->data[0] = sync_seq;
}

bool lbr_can_decode_time_sync(const lbr_can_frame_t *f, uint8_t *sync_seq, uint16_t *epoch)
{
    if (f->id != LBR_CAN_ID_TIME_SYNC || f->dlc != 8) {
        return false;
    }
    *sync_seq = f->data[4];
    *epoch    = lbr_get_u16(f->data + 5);
    return true;
}

bool lbr_can_decode_time_follow_up(const lbr_can_frame_t *f, uint8_t *sync_seq, uint64_t *tx_us)
{
    if (f->id != LBR_CAN_ID_TIME_FOLLOW_UP || f->dlc != 8) {
        return false;
    }
    *sync_seq = f->data[0];
    *tx_us    = lbr_get_u64(f->data) >> 8;
    return true;
}

void lbr_can_encode_command(uint8_t code, const uint8_t args[7], lbr_can_frame_t *out)
{
    out->id      = LBR_CAN_ID_COMMAND;
    out->dlc     = 8;
    out->data[0] = code;
    memcpy(out->data + 1, args, 7);
}

bool lbr_can_decode_command(const lbr_can_frame_t *f, uint8_t *code, uint8_t args[7])
{
    if (f->id != LBR_CAN_ID_COMMAND || f->dlc != 8) {
        return false;
    }
    *code = f->data[0];
    memcpy(args, f->data + 1, 7);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Heartbeat and health                                                      */

bool lbr_can_encode_heartbeat(uint8_t node, uint8_t state, uint8_t err_flags, lbr_can_frame_t *out)
{
    if (!lbr_node_is_can(node)) {
        return false;
    }
    out->id  = LBR_CAN_ID_HEARTBEAT_BASE + node;
    out->dlc = 2;
    memset(out->data, 0, sizeof out->data);
    out->data[0] = state;
    out->data[1] = err_flags;
    return true;
}

uint8_t lbr_can_decode_heartbeat(const lbr_can_frame_t *f, uint8_t *state, uint8_t *err_flags)
{
    if (f->id <= LBR_CAN_ID_HEARTBEAT_BASE ||
        f->id > LBR_CAN_ID_HEARTBEAT_BASE + LBR_NODE_CAN_MAX || f->dlc != 2) {
        return 0;
    }
    *state     = f->data[0];
    *err_flags = f->data[1];
    return (uint8_t)(f->id - LBR_CAN_ID_HEARTBEAT_BASE);
}

bool lbr_can_encode_health_page(uint8_t node, const lbr_health_record_t *h, unsigned page,
                                lbr_can_frame_t *out)
{
    if (page >= LBR_HEALTH_PAGE_COUNT || !lbr_node_is_can(node)) {
        return false;
    }
    out->id  = LBR_CAN_ID_HEALTH_BASE + 16u * node + page;
    out->dlc = 8;
    return lbr_health_page_encode(h, page, out->data);
}

int lbr_can_decode_health_page(const lbr_can_frame_t *f, lbr_health_record_t *h)
{
    if (f->id < LBR_CAN_ID_HEALTH_BASE || f->id > LBR_CAN_ID_MAX || f->dlc != 8) {
        return -1;
    }
    uint32_t node = (f->id - LBR_CAN_ID_HEALTH_BASE) >> 4;
    unsigned page = f->id & 0xFu;
    if (!lbr_node_is_can((uint8_t)node) || page >= LBR_HEALTH_PAGE_COUNT ||
        !lbr_health_page_decode(h, page, f->data)) {
        return -1;
    }
    h->node_id = (uint8_t)node;
    return (int)page;
}

/* ------------------------------------------------------------------------ */
/* Bus decoder                                                               */

void lbr_bus_decoder_init(lbr_bus_decoder_t *d)
{
    memset(d, 0, sizeof *d);
}

static bool can_node_synced(const lbr_bus_decoder_t *d, uint8_t node)
{
    if (node == LBR_NODE_CENTRAL) {
        return true;
    }
    return lbr_node_is_can(node) && d->hb_seen[node] &&
           (d->hb_flags[node] & LBR_HB_ERR_NOT_SYNCED) == 0;
}

bool lbr_bus_decoder_synced(const lbr_bus_decoder_t *d, uint8_t origin)
{
    int corner = lbr_node_corner_index(origin);
    if (corner >= 0) {
        return d->corner_seen && (d->corner_synced & (1u << corner)) != 0 &&
               can_node_synced(d, LBR_NODE_IMU);
    }
    return can_node_synced(d, origin);
}

size_t lbr_bus_decode(lbr_bus_decoder_t *d, const lbr_can_frame_t *f, uint64_t rx_shared_us,
                      lbr_sample_t *out, size_t cap)
{
    uint8_t state, flags;
    uint8_t hb = lbr_can_decode_heartbeat(f, &state, &flags);
    if (hb != 0) {
        d->hb_seen[hb]  = true;
        d->hb_flags[hb] = flags;
        return 0;
    }
    if (cap > 0 && lbr_can_decode_event(f, rx_shared_us, out)) {
        return 1;
    }
    const lbr_frame_def_t *def = lbr_frame_def((uint16_t)f->id);
    if (def == NULL) {
        return 0;
    }
    size_t n =
        lbr_can_decode_data(f, rx_shared_us, lbr_bus_decoder_synced(d, def->origin), out, cap);
    for (size_t i = 0; i < n; i++) {
        if (out[i].signal == LBR_SIG_CORNER_SYNCED) {
            d->corner_seen   = true;
            d->corner_synced = out[i].valid ? (uint8_t)out[i].v.u32 : 0u;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* Raw frame log record                                                      */

size_t lbr_can_frame_rec_encode(uint64_t t_us, uint8_t flags, const lbr_can_frame_t *f,
                                uint8_t *buf, size_t cap)
{
    if (cap < LBR_CAN_FRAME_REC_SIZE || f->dlc > 8) {
        return 0;
    }
    lbr_put_u64(buf, t_us);
    lbr_put_u32(buf + 8, f->id);
    buf[12] = flags;
    buf[13] = f->dlc;
    memset(buf + 14, 0, 8);
    memcpy(buf + 14, f->data, f->dlc);
    return LBR_CAN_FRAME_REC_SIZE;
}

size_t lbr_can_frame_rec_decode(const uint8_t *buf, size_t len, uint64_t *t_us, uint8_t *flags,
                                lbr_can_frame_t *f)
{
    if (len < LBR_CAN_FRAME_REC_SIZE || buf[13] > 8) {
        return 0;
    }
    *t_us  = lbr_get_u64(buf);
    *flags = buf[12];
    f->id  = lbr_get_u32(buf + 8);
    f->dlc = buf[13];
    memcpy(f->data, buf + 14, 8);
    return LBR_CAN_FRAME_REC_SIZE;
}
