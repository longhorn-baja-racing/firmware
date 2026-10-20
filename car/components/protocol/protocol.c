#include "lbr/protocol.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/frames.h"
#include "lbr/nodes.h"

/* ------------------------------------------------------------------------ */
/* Catalog                                                                   */

static const lbr_signal_info_t s_catalog[] = {
#define LBR_SIGNAL(name_, id_, type_, unit_, desc_)                                                \
    {                                                                                              \
        .id   = LBR_SIG_##name_,                                                                   \
        .name = #name_,                                                                            \
        .type = LBR_VAL_##type_,                                                                   \
        .unit = LBR_UNIT_##unit_,                                                                  \
        .desc = (desc_),                                                                           \
    },
#include "lbr/signals.def"
#undef LBR_SIGNAL
};

#define CATALOG_LEN (sizeof s_catalog / sizeof s_catalog[0])

const lbr_signal_info_t *lbr_signal_info(lbr_signal_id_t id)
{
    /* Catalog is sorted by id (enforced by the host tests): binary search. */
    size_t lo = 0;
    size_t hi = CATALOG_LEN;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (s_catalog[mid].id == id) {
            return &s_catalog[mid];
        }
        if (s_catalog[mid].id < id) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return NULL;
}

size_t lbr_signal_count(void)
{
    return CATALOG_LEN;
}

const lbr_signal_info_t *lbr_signal_at(size_t index)
{
    return index < CATALOG_LEN ? &s_catalog[index] : NULL;
}

const char *lbr_unit_name(lbr_unit_t unit)
{
    static const char *const names[LBR_UNIT_COUNT_] = {
        [LBR_UNIT_NONE] = "",           [LBR_UNIT_M_PER_S2] = "m/s^2",
        [LBR_UNIT_RAD_PER_S] = "rad/s", [LBR_UNIT_DEG_C] = "degC",
        [LBR_UNIT_RPM] = "rpm",         [LBR_UNIT_M_PER_S] = "m/s",
        [LBR_UNIT_PERCENT] = "%",       [LBR_UNIT_DEG_E7] = "deg*1e-7",
        [LBR_UNIT_METER] = "m",         [LBR_UNIT_DEGREE] = "deg",
        [LBR_UNIT_COUNT] = "count",     [LBR_UNIT_MICROSTRAIN] = "ue",
        [LBR_UNIT_UNIX_S] = "unix_s",   [LBR_UNIT_BITMASK] = "bitmask",
    };
    return ((unsigned)unit < LBR_UNIT_COUNT_) ? names[unit] : "?";
}

const char *lbr_value_type_name(lbr_value_type_t type)
{
    static const char *const names[LBR_VAL_COUNT_] = {
        [LBR_VAL_NONE] = "none", [LBR_VAL_F32] = "f32",   [LBR_VAL_I32] = "i32",
        [LBR_VAL_U32] = "u32",   [LBR_VAL_BOOL] = "bool", [LBR_VAL_STATUS] = "status",
        [LBR_VAL_RAW] = "raw",
    };
    return ((unsigned)type < LBR_VAL_COUNT_) ? names[type] : "?";
}

const char *lbr_unavail_reason_name(lbr_unavail_reason_t why)
{
    switch (why) {
        case LBR_UNAVAIL_UNSPECIFIED:
            return "UNSPECIFIED";
        case LBR_UNAVAIL_NOT_FITTED:
            return "NOT_FITTED";
        case LBR_UNAVAIL_NO_DATA:
            return "NO_DATA";
        case LBR_UNAVAIL_SENSOR_FAULT:
            return "SENSOR_FAULT";
        case LBR_UNAVAIL_OUT_OF_RANGE:
            return "OUT_OF_RANGE";
        case LBR_UNAVAIL_NOT_CALIBRATED:
            return "NOT_CALIBRATED";
        case LBR_UNAVAIL_BUS_ERROR:
            return "BUS_ERROR";
        default:
            return "?";
    }
}

const char *lbr_event_name(lbr_event_code_t code)
{
    switch (code) {
        case LBR_EVT_BOOT:
            return "BOOT";
        case LBR_EVT_SESSION_START:
            return "SESSION_START";
        case LBR_EVT_TIME_SYNCED:
            return "TIME_SYNCED";
        case LBR_EVT_TIME_STEP:
            return "TIME_STEP";
        case LBR_EVT_CONFIG_INVALID:
            return "CONFIG_INVALID";
        case LBR_EVT_SENSOR_INIT_FAIL:
            return "SENSOR_INIT_FAIL";
        case LBR_EVT_SENSOR_RECOVERED:
            return "SENSOR_RECOVERED";
        case LBR_EVT_CAN_BUS_OFF:
            return "CAN_BUS_OFF";
        case LBR_EVT_CAN_RECOVERED:
            return "CAN_RECOVERED";
        case LBR_EVT_SD_MISSING:
            return "SD_MISSING";
        case LBR_EVT_SD_WRITE_ERROR:
            return "SD_WRITE_ERROR";
        case LBR_EVT_SD_MOUNTED:
            return "SD_MOUNTED";
        case LBR_EVT_QUEUE_OVERFLOW:
            return "QUEUE_OVERFLOW";
        case LBR_EVT_NODE_LOST:
            return "NODE_LOST";
        case LBR_EVT_NODE_FOUND:
            return "NODE_FOUND";
        case LBR_EVT_CAN_PINS_UNSET:
            return "CAN_PINS_UNSET";
        default:
            return "?";
    }
}

uint32_t lbr_catalog_hash(void)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < CATALOG_LEN; i++) {
        const lbr_signal_info_t *s = &s_catalog[i];
        uint8_t                  rec[4];
        lbr_put_u16(rec, (uint16_t)s->id);
        rec[2] = (uint8_t)s->type;
        rec[3] = (uint8_t)s->unit;
        crc    = lbr_crc32_update(crc, rec, sizeof rec);
        crc    = lbr_crc32_update(crc, s->name, strlen(s->name) + 1);
    }
    return lbr_frames_hash(crc);
}

/* ------------------------------------------------------------------------ */
/* Sample codec                                                              */

#define FLAG_VALID     0x10u
#define FLAG_SYNCED    0x20u
#define FLAG_SEQ_INFER 0x40u
#define FLAG_RESERVED  0x80u
#define TYPE_MASK      0x0Fu

uint32_t lbr_sample_value_bits(const lbr_sample_t *s)
{
    if (!s->valid) {
        return s->v.u32;
    }
    switch (s->type) {
        case LBR_VAL_F32:
            return lbr_f32_bits(s->v.f32);
        case LBR_VAL_I32:
            return (uint32_t)s->v.i32;
        case LBR_VAL_BOOL:
            return s->v.b ? 1u : 0u;
        default:
            return s->v.u32;
    }
}

void lbr_sample_set_value_bits(lbr_sample_t *s, uint32_t bits)
{
    if (s->valid && s->type == LBR_VAL_F32) {
        s->v.f32 = lbr_bits_f32(bits);
    } else if (s->valid && s->type == LBR_VAL_I32) {
        s->v.i32 = (int32_t)bits;
    } else if (s->valid && s->type == LBR_VAL_BOOL) {
        s->v.b = bits != 0;
    } else {
        s->v.u32 = bits;
    }
}

size_t lbr_sample_encode(const lbr_sample_t *s, uint8_t *buf, size_t cap)
{
    if (s == NULL || buf == NULL || cap < LBR_SAMPLE_WIRE_SIZE ||
        (unsigned)s->type >= LBR_VAL_COUNT_) {
        return 0;
    }
    uint8_t tf = (uint8_t)s->type;
    tf |= s->valid ? FLAG_VALID : 0u;
    tf |= s->synced ? FLAG_SYNCED : 0u;
    tf |= s->seq_inferred ? FLAG_SEQ_INFER : 0u;

    lbr_put_u16(buf + 0, (uint16_t)s->signal);
    buf[2] = s->source;
    buf[3] = tf;
    lbr_put_u32(buf + 4, s->seq);
    lbr_put_u64(buf + 8, s->t_mono_us);
    lbr_put_u32(buf + 16, lbr_sample_value_bits(s));
    return LBR_SAMPLE_WIRE_SIZE;
}

size_t lbr_sample_decode(const uint8_t *buf, size_t len, lbr_sample_t *out)
{
    if (buf == NULL || out == NULL || len < LBR_SAMPLE_WIRE_SIZE) {
        return 0;
    }
    uint8_t tf = buf[3];
    if ((tf & FLAG_RESERVED) || (tf & TYPE_MASK) >= LBR_VAL_COUNT_) {
        return 0;
    }
    lbr_sample_t s = {
        .signal       = (lbr_signal_id_t)lbr_get_u16(buf + 0),
        .source       = buf[2],
        .type         = (lbr_value_type_t)(tf & TYPE_MASK),
        .valid        = (tf & FLAG_VALID) != 0,
        .synced       = (tf & FLAG_SYNCED) != 0,
        .seq_inferred = (tf & FLAG_SEQ_INFER) != 0,
        .seq          = lbr_get_u32(buf + 4),
        .t_mono_us    = lbr_get_u64(buf + 8),
    };
    lbr_sample_set_value_bits(&s, lbr_get_u32(buf + 16));
    *out = s;
    return LBR_SAMPLE_WIRE_SIZE;
}

/* ------------------------------------------------------------------------ */
/* Node names                                                                */

const char *lbr_node_name(uint8_t node_id)
{
    switch (node_id) {
        case LBR_NODE_IMU:
            return "imu";
        case LBR_NODE_POWERTRAIN:
            return "powertrain";
        case LBR_NODE_DRIVER_INPUT:
            return "driver_input";
        case LBR_NODE_TEMPERATURE:
            return "temperature";
        case LBR_NODE_STRESS:
            return "stress";
        case LBR_NODE_CENTRAL:
            return "central";
        case LBR_NODE_DASHBOARD:
            return "dashboard";
        case LBR_NODE_CORNER_FL:
            return "corner_fl";
        case LBR_NODE_CORNER_FR:
            return "corner_fr";
        case LBR_NODE_CORNER_BL:
            return "corner_bl";
        case LBR_NODE_CORNER_BR:
            return "corner_br";
        default:
            return "?";
    }
}
