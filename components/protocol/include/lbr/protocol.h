#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/signals.h"

/* Bump on any incompatible change; must match lbr-dashboard. */
#define LBR_PROTOCOL_VERSION 1u

/*
 * Value representation for a single sample. The signal id (schema-driven) tells
 * the consumer the unit and meaning; `type` tells it how to read the union.
 */
typedef enum {
    LBR_VAL_NONE = 0, /* explicitly unavailable — never substitute zero */
    LBR_VAL_F32,
    LBR_VAL_I32,
    LBR_VAL_U32,
    LBR_VAL_BOOL,
    LBR_VAL_STATUS, /* enumerated status / calibration code */
} lbr_value_type_t;

typedef struct {
    lbr_signal_id_t  signal;    /* what */
    uint64_t         t_mono_us; /* when: shared monotonic time base, microseconds */
    uint32_t         seq;       /* per-stream monotonic sequence number */
    lbr_value_type_t type;
    bool             valid;     /* false => unavailable; consumers MUST honor this */
    union {
        float    f32;
        int32_t  i32;
        uint32_t u32;
        bool     b;
        uint32_t status;
    } v;
} lbr_sample_t;

/*
 * Encode/decode a sample to/from the shared wire/log byte layout. The exact
 * layout and checksums are defined by the shared spec and MUST match
 * lbr-dashboard. Returns bytes written/consumed, or 0 on error.
 *
 * TODO: implement once the shared byte layout is frozen (see docs/protocol.md).
 */
size_t lbr_sample_encode(const lbr_sample_t *s, uint8_t *buf, size_t cap);
size_t lbr_sample_decode(const uint8_t *buf, size_t len, lbr_sample_t *out);
