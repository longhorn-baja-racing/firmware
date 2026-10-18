#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Consistent Overhead Byte Stuffing. Encoded data never contains 0x00, so a
 * 0x00 byte delimits frames on byte streams (USB serial, WiFi TCP) and a
 * receiver can always resynchronize at the next delimiter after corruption.
 */

/* Worst-case encoded size for `len` input bytes (excluding the delimiter). */
#define LBR_COBS_MAX_ENCODED(len) ((len) + ((len) / 254u) + 1u)

/* Returns encoded length (no delimiter appended), or 0 if `cap` is too small. */
size_t lbr_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t cap);

/* Returns decoded length, or 0 on malformed input / insufficient `cap`. An
 * empty frame also decodes to 0; callers treat it as "nothing". */
size_t lbr_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap);

/*
 * Streaming receiver: feed bytes one at a time. When a delimiter completes a
 * frame, lbr_cobs_rx_push returns the decoded length (> 0) and the frame is
 * in `out`. Oversized or malformed frames are counted and discarded.
 */
typedef struct {
    uint8_t *buf; /* raw (still-encoded) accumulation buffer */
    size_t   cap;
    size_t   len;
    bool     overflow;
    uint32_t frames_ok;
    uint32_t frames_bad; /* malformed or oversized: never silently dropped */
} lbr_cobs_rx_t;

void   lbr_cobs_rx_init(lbr_cobs_rx_t *rx, uint8_t *buf, size_t cap);
size_t lbr_cobs_rx_push(lbr_cobs_rx_t *rx, uint8_t byte, uint8_t *out, size_t out_cap);
