#include "lbr/can.h"

#include <string.h>

#include "lbr/bytes.h"

#define T_LO_MASK  0x00FFFFFFu
#define T_LO_RANGE 0x01000000

#define SEQ_MASK  ((1u << LBR_CAN_SEQ_BITS) - 1u)
#define B7_SYNCED 0x40u
#define B7_VALID  0x80u

bool lbr_can_encode_sample(const lbr_sample_t *s, lbr_can_frame_t *out)
{
    const lbr_signal_info_t *info = lbr_signal_info(s->signal);
    if (info == NULL || s->source == 0) {
        return false;
    }
    out->id  = lbr_can_id_make(info->can_prio, LBR_CAN_KIND_SAMPLE, s->source, (uint16_t)s->signal);
    out->dlc = 8;
    lbr_put_u32(out->data, lbr_sample_value_bits(s));
    lbr_put_u24(out->data + 4, (uint32_t)(s->t_mono_us & T_LO_MASK));
    out->data[7] =
        (uint8_t)((s->seq & SEQ_MASK) | (s->synced ? B7_SYNCED : 0u) | (s->valid ? B7_VALID : 0u));
    return true;
}

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

bool lbr_can_decode_sample(const lbr_can_frame_t *f, uint64_t rx_shared_us, lbr_sample_t *out)
{
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    if (id.kind != LBR_CAN_KIND_SAMPLE || f->dlc != 8 || id.source == 0) {
        return false;
    }
    const lbr_signal_info_t *info = lbr_signal_info((lbr_signal_id_t)id.sub);
    uint8_t                  b7   = f->data[7];

    lbr_sample_t s = {
        .signal    = (lbr_signal_id_t)id.sub,
        .source    = id.source,
        .type      = info ? info->type : LBR_VAL_RAW,
        .valid     = (b7 & B7_VALID) != 0,
        .synced    = (b7 & B7_SYNCED) != 0,
        .seq_short = true,
        .seq       = b7 & SEQ_MASK,
        .t_mono_us = lbr_can_unwrap_time(lbr_get_u24(f->data + 4), rx_shared_us),
    };
    lbr_sample_set_value_bits(&s, lbr_get_u32(f->data));
    *out = s;
    return true;
}

void lbr_can_encode_time_sync(uint8_t source, uint8_t sync_seq, uint16_t epoch,
                              lbr_can_frame_t *out)
{
    out->id  = lbr_can_id_make(LBR_CAN_PRIO_TIME, LBR_CAN_KIND_SYSTEM, source, LBR_SYS_TIME_SYNC);
    out->dlc = 3;
    memset(out->data, 0, sizeof out->data);
    out->data[0] = sync_seq;
    lbr_put_u16(out->data + 1, epoch);
}

void lbr_can_encode_time_follow_up(uint8_t source, uint8_t sync_seq, uint64_t tx_us,
                                   lbr_can_frame_t *out)
{
    out->id =
        lbr_can_id_make(LBR_CAN_PRIO_TIME, LBR_CAN_KIND_SYSTEM, source, LBR_SYS_TIME_FOLLOW_UP);
    out->dlc = 8;
    lbr_put_u64(out->data, tx_us << 8);
    out->data[0] = sync_seq;
}

static bool is_system(const lbr_can_frame_t *f, uint16_t msg)
{
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    return id.kind == LBR_CAN_KIND_SYSTEM && id.sub == msg;
}

bool lbr_can_decode_time_sync(const lbr_can_frame_t *f, uint8_t *sync_seq, uint16_t *epoch)
{
    if (!is_system(f, LBR_SYS_TIME_SYNC) || f->dlc < 3) {
        return false;
    }
    *sync_seq = f->data[0];
    *epoch    = lbr_get_u16(f->data + 1);
    return true;
}

bool lbr_can_decode_time_follow_up(const lbr_can_frame_t *f, uint8_t *sync_seq, uint64_t *tx_us)
{
    if (!is_system(f, LBR_SYS_TIME_FOLLOW_UP) || f->dlc != 8) {
        return false;
    }
    *sync_seq = f->data[0];
    *tx_us    = lbr_get_u64(f->data) >> 8;
    return true;
}

bool lbr_can_encode_health_page(uint8_t source, const lbr_health_record_t *h, unsigned page,
                                lbr_can_frame_t *out)
{
    if (page >= LBR_HEALTH_PAGE_COUNT) {
        return false;
    }
    out->id  = lbr_can_id_make(LBR_CAN_PRIO_HEALTH, LBR_CAN_KIND_SYSTEM, source,
                               (uint16_t)(LBR_SYS_HEALTH_BASE + page));
    out->dlc = 8;
    return lbr_health_page_encode(h, page, out->data);
}

int lbr_can_decode_health_page(const lbr_can_frame_t *f, lbr_health_record_t *h)
{
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    if (id.kind != LBR_CAN_KIND_SYSTEM || id.sub < LBR_SYS_HEALTH_BASE ||
        id.sub >= LBR_SYS_HEALTH_BASE + LBR_HEALTH_PAGE_COUNT || f->dlc != 8) {
        return -1;
    }
    unsigned page = id.sub - LBR_SYS_HEALTH_BASE;
    if (!lbr_health_page_decode(h, page, f->data)) {
        return -1;
    }
    h->node_id = id.source;
    return (int)page;
}

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
