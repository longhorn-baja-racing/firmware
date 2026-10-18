#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/signals.h"

/*
 * Shared protocol definitions (firmware side of the spec in docs/protocol.md).
 * Every byte layout here is little-endian and MUST match lbr-dashboard; the
 * golden fixtures in tests/fixtures pin it down.
 */

/* Bump on any incompatible change; must match lbr-dashboard. */
#define LBR_PROTOCOL_VERSION 2u

/* Value representation. The catalog fixes the type of every known signal;
 * LBR_VAL_RAW is only produced by a decoder that does not know the signal. */
typedef enum {
    LBR_VAL_NONE = 0, /* no value representation (reserved) */
    LBR_VAL_F32,
    LBR_VAL_I32,
    LBR_VAL_U32,
    LBR_VAL_BOOL,
    LBR_VAL_STATUS, /* enumerated status / calibration / event code */
    LBR_VAL_RAW,    /* signal unknown to this decoder: 32 value bits preserved */
    LBR_VAL_COUNT_
} lbr_value_type_t;

/* Canonical units (display metadata lives in the dashboard). */
typedef enum {
    LBR_UNIT_NONE = 0,
    LBR_UNIT_M_PER_S2,
    LBR_UNIT_RAD_PER_S,
    LBR_UNIT_DEG_C,
    LBR_UNIT_RPM,
    LBR_UNIT_M_PER_S,
    LBR_UNIT_PERCENT,
    LBR_UNIT_VOLT,
    LBR_UNIT_AMP,
    LBR_UNIT_MICROTESLA,
    LBR_UNIT_DEG_E7,
    LBR_UNIT_METER,
    LBR_UNIT_DEGREE,
    LBR_UNIT_COUNT,
    LBR_UNIT_KPA,
    LBR_UNIT_MM,
    LBR_UNIT_MICROSTRAIN,
    LBR_UNIT_LITER,
    LBR_UNIT_UNIX_S,
    LBR_UNIT_BITMASK,
    LBR_UNIT_COUNT_
} lbr_unit_t;

/* Why a sample is unavailable. Carried in the value slot when valid == false,
 * so "no data" is always explicit and never confused with a zero reading. */
typedef enum {
    LBR_UNAVAIL_UNSPECIFIED = 0,
    LBR_UNAVAIL_NOT_FITTED,     /* no driver/part configured for this channel */
    LBR_UNAVAIL_NO_DATA,        /* driver alive but no fresh reading this tick */
    LBR_UNAVAIL_SENSOR_FAULT,   /* driver reported a fault / init failed */
    LBR_UNAVAIL_OUT_OF_RANGE,   /* reading outside the physically valid range */
    LBR_UNAVAIL_NOT_CALIBRATED, /* raw reading exists but no calibration */
    LBR_UNAVAIL_BUS_ERROR,      /* I2C/SPI/ADC transaction failed */
} lbr_unavail_reason_t;

/* Event / fault codes carried by LBR_SIG_EVENT and LBR_SIG_FAULT samples:
 * value = code | (arg << 16). Append only. */
typedef enum {
    LBR_EVT_BOOT             = 0x0001, /* arg: lbr_reset_reason_t */
    LBR_EVT_SESSION_START    = 0x0002,
    LBR_EVT_TIME_SYNCED      = 0x0003,
    LBR_EVT_TIME_STEP        = 0x0004, /* time base stepped instead of slewed */
    LBR_EVT_CONFIG_INVALID   = 0x0005, /* acquisition refused to start */
    LBR_EVT_SENSOR_INIT_FAIL = 0x0010, /* arg: sensor index */
    LBR_EVT_SENSOR_RECOVERED = 0x0011, /* arg: sensor index */
    LBR_EVT_CAN_BUS_OFF      = 0x0020,
    LBR_EVT_CAN_RECOVERED    = 0x0021,
    LBR_EVT_SD_MISSING       = 0x0030,
    LBR_EVT_SD_WRITE_ERROR   = 0x0031,
    LBR_EVT_SD_MOUNTED       = 0x0032,
    LBR_EVT_QUEUE_OVERFLOW   = 0x0040, /* arg: queue id */
    LBR_EVT_NODE_LOST        = 0x0050, /* arg: node id (gateway only) */
    LBR_EVT_NODE_FOUND       = 0x0051, /* arg: node id (gateway only) */
    LBR_EVT_CAN_PINS_UNSET   = 0x0060, /* CAN GPIO still TBD in Kconfig */
} lbr_event_code_t;

/* Spec names for enum values (ASCII, upper case); "?" if unknown. */
const char *lbr_unavail_reason_name(lbr_unavail_reason_t why);
const char *lbr_event_name(lbr_event_code_t code);

static inline uint32_t lbr_event_value(lbr_event_code_t code, uint16_t arg)
{
    return (uint32_t)code | ((uint32_t)arg << 16);
}

/*
 * One measurement. `seq` is a per-stream monotonic counter; a stream is
 * (source node, signal, medium). On CAN only its low 6 bits travel and the
 * decoded sample sets `seq_short` so consumers know to extend it.
 */
typedef struct {
    lbr_signal_id_t  signal;    /* what */
    uint8_t          source;    /* node id that produced it */
    lbr_value_type_t type;      /* how to read `v` */
    bool             valid;     /* false => unavailable; v.u32 holds lbr_unavail_reason_t */
    bool             synced;    /* timestamp is on the shared (gateway) time base */
    bool             seq_short; /* seq carries only 6 bits (CAN-sourced) */
    uint32_t         seq;       /* per-stream monotonic sequence number */
    uint64_t         t_mono_us; /* when: shared monotonic time base, microseconds */
    union {
        float    f32;
        int32_t  i32;
        uint32_t u32;
        bool     b;
        uint32_t status;
    } v;
} lbr_sample_t;

static inline void lbr_sample_set_unavailable(lbr_sample_t *s, lbr_unavail_reason_t why)
{
    s->valid = false;
    s->v.u32 = (uint32_t)why;
}

/* Raw 32 value bits as carried on the wire (bool is 0/1). */
uint32_t lbr_sample_value_bits(const lbr_sample_t *s);
void     lbr_sample_set_value_bits(lbr_sample_t *s, uint32_t bits);

/* ------------------------------------------------------------------------ */
/* Catalog                                                                   */

typedef struct {
    const char      *name; /* catalog NAME, e.g. "ACCEL_X" */
    const char      *desc;
    lbr_signal_id_t  id;
    lbr_value_type_t type;
    lbr_unit_t       unit;
    uint8_t          can_prio; /* 0 highest .. 7 lowest */
} lbr_signal_info_t;

/* NULL if the id is not in this build's catalog. */
const lbr_signal_info_t *lbr_signal_info(lbr_signal_id_t id);
size_t                   lbr_signal_count(void);
const lbr_signal_info_t *lbr_signal_at(size_t index);

const char *lbr_unit_name(lbr_unit_t unit);
const char *lbr_value_type_name(lbr_value_type_t type);

/* CRC-32 over a canonical serialization of the catalog (id, type, unit, prio,
 * name). Travels in heartbeats and session metadata so a node or dashboard
 * built against a different catalog is detected, not silently misdecoded. */
uint32_t lbr_catalog_hash(void);

/* ------------------------------------------------------------------------ */
/* Record types shared by the log format and transport packets               */

typedef enum {
    LBR_REC_SESSION     = 0x01, /* log only: first record of every log */
    LBR_REC_SAMPLE      = 0x02, /* LBR_SAMPLE_WIRE_SIZE bytes */
    LBR_REC_HEALTH      = 0x03, /* LBR_HEALTH_WIRE_SIZE bytes (health.h) */
    LBR_REC_CAN_FRAME   = 0x04, /* LBR_CAN_FRAME_REC_SIZE bytes (can.h) */
    LBR_REC_LOSS        = 0x05, /* records/bytes dropped before this point */
    LBR_REC_TEXT        = 0x06, /* ASCII diagnostic text */
    LBR_REC_END         = 0x07, /* log only: clean close marker */
    LBR_REC_HEALTH_LITE = 0x08, /* transport only: compact LoRa health */
    LBR_REC_FILE_INFO   = 0x10, /* transport only: log listing entry */
    LBR_REC_FILE_CHUNK  = 0x11, /* transport only: log dump data */
    LBR_REC_FILE_END    = 0x12, /* transport only: dump complete */
} lbr_record_type_t;

/*
 * Sample wire/log encoding, 20 bytes:
 *
 *   off size field
 *   0   2    signal id
 *   2   1    source node id
 *   3   1    bits 0..3 value type, bit 4 valid, bit 5 synced, bit 6 seq_short,
 *            bit 7 reserved (0)
 *   4   4    seq
 *   8   8    t_mono_us
 *   16  4    value bits (f32 IEEE-754 / i32 / u32 / bool 0|1 / status), or the
 *            lbr_unavail_reason_t when not valid
 *
 * Returns bytes written/consumed, or 0 on error (short buffer, unknown value
 * type, reserved bit set).
 */
#define LBR_SAMPLE_WIRE_SIZE 20u

size_t lbr_sample_encode(const lbr_sample_t *s, uint8_t *buf, size_t cap);
size_t lbr_sample_decode(const uint8_t *buf, size_t len, lbr_sample_t *out);

/* ------------------------------------------------------------------------ */
/* Sequence numbers                                                          */

/* Signed distance from `prev` to `next` in modulo-2^32 sequence space:
 * 1 = next in order, 0 = duplicate, >1 = gap of (d - 1), <0 = reordered. */
static inline int32_t lbr_seq_delta(uint32_t prev, uint32_t next)
{
    return (int32_t)(next - prev);
}

/* Same for an n-bit wrapped counter (n <= 31), e.g. the 6-bit CAN seq. */
static inline int32_t lbr_seq_delta_bits(uint32_t prev, uint32_t next, unsigned bits)
{
    uint32_t mask = (1u << bits) - 1u;
    uint32_t d    = (next - prev) & mask;
    return (d & (1u << (bits - 1))) ? (int32_t)d - (int32_t)(mask + 1u) : (int32_t)d;
}
