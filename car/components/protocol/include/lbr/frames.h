#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/signals.h"

/*
 * CAN data frame table: the schema behind the 11-bit id map in
 * DAQ-WIRING.md section 7 and docs/can-ids.md. Which signals travel in which
 * frame, at which byte, in which integer encoding and scale, is data here,
 * exported to catalog.json and hashed into the catalog hash, so the
 * dashboard decodes frames from the table instead of a hand-written layout.
 *
 * Time grid. Every data frame belongs to a stream sampled at a fixed rate on
 * the shared time base: sample k of a stream is taken at shared time
 * k * period_us (period_us = 1000000 / rate_hz). A frame's 16-bit sample
 * counter is k modulo 65536, so a receiver rebuilds the exact sample time
 * from the counter and its own clock. Frames without a counter are stamped
 * at the last grid point at or before reception (lbr/can.h).
 *
 * Unavailable values. Each integer encoding reserves its 8 lowest (signed)
 * or highest (unsigned) raw values: raw = base + lbr_unavail_reason_t. A
 * missing reading is therefore always explicit, never a zero. Readings that
 * do not fit the remaining range travel as OUT_OF_RANGE.
 */

typedef enum {
    LBR_ENC_I16 = 0, /* unavailable: -32768 .. -32761 */
    LBR_ENC_U16,     /* unavailable: 0xFFF8 .. 0xFFFF */
    LBR_ENC_U8,      /* unavailable: 0xF8 .. 0xFF */
    LBR_ENC_I32,     /* unavailable: INT32_MIN .. INT32_MIN + 7 */
    LBR_ENC_U32,     /* unavailable: 0xFFFFFFF8 .. 0xFFFFFFFF */
    LBR_ENC_COUNT_
} lbr_field_enc_t;

#define LBR_ENC_UNAVAIL_SLOTS 8u

typedef struct {
    lbr_signal_id_t signal;
    uint8_t         offset; /* first payload byte, little-endian */
    uint8_t         enc;    /* lbr_field_enc_t */
    float           scale;  /* physical value (catalog unit) = raw * scale; 1 for integer signals */
} lbr_field_def_t;

#define LBR_FRAME_MAX_FIELDS 8u
#define LBR_FRAME_NO_COUNTER 0xFFu

typedef struct {
    uint16_t        can_id; /* 11-bit standard id; lower wins arbitration */
    const char     *name;
    uint8_t         sender;         /* CAN node that transmits it (lbr_node_id_t) */
    uint8_t         origin;         /* node that takes the samples (a corner for forwarded IMU) */
    uint16_t        rate_hz;        /* stream rate; 1000000 is a multiple of it */
    uint8_t         dlc;            /* payload bytes */
    uint8_t         counter_offset; /* uint16 sample counter, or LBR_FRAME_NO_COUNTER */
    uint8_t         n_fields;
    lbr_field_def_t fields[LBR_FRAME_MAX_FIELDS];
} lbr_frame_def_t;

/* NULL if `can_id` is not a data frame of this build's table. */
const lbr_frame_def_t *lbr_frame_def(uint16_t can_id);
size_t                 lbr_frame_count(void);
const lbr_frame_def_t *lbr_frame_at(size_t index);

/* The frame (and field index) carrying `signal`, or NULL. Each signal
 * travels in exactly one frame (checked by the host tests). */
const lbr_frame_def_t *lbr_frame_for_signal(lbr_signal_id_t signal, size_t *field_index);

static inline uint32_t lbr_frame_period_us(const lbr_frame_def_t *d)
{
    return 1000000u / d->rate_hz;
}

/* Bytes of an encoding (2, 2, 1, 4, 4). */
unsigned    lbr_field_enc_size(lbr_field_enc_t enc);
const char *lbr_field_enc_name(lbr_field_enc_t enc);

/* Fold the frame table into a running catalog CRC (lbr_catalog_hash). */
uint32_t lbr_frames_hash(uint32_t crc);
