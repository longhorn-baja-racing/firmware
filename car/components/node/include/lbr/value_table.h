#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/protocol.h"

/*
 * Latest value per watched signal, with staleness. Used by consumers that
 * show "the current value" (cockpit display, walk-up status): a value older
 * than `stale_us` or marked invalid reads as unavailable, never as a number.
 * Pure; host-tested.
 */

#define LBR_VALUE_TABLE_MAX 32u

typedef struct {
    lbr_signal_id_t signal;
    uint8_t         source; /* 0 = any node */
    bool            have;
    lbr_sample_t    last;
    uint64_t        rx_us;
} lbr_value_entry_t;

typedef struct {
    lbr_value_entry_t entries[LBR_VALUE_TABLE_MAX];
    size_t            n;
    uint64_t          stale_us;
} lbr_value_table_t;

void lbr_value_table_init(lbr_value_table_t *t, uint64_t stale_us);

/* Watch a signal (from `source`, or any source if 0). False if full. */
bool lbr_value_table_watch(lbr_value_table_t *t, lbr_signal_id_t signal, uint8_t source);

/* Offer a decoded sample; stored if watched. */
void lbr_value_table_offer(lbr_value_table_t *t, const lbr_sample_t *s, uint64_t rx_us);

typedef enum {
    LBR_VALUE_NONE = 0, /* never received */
    LBR_VALUE_STALE,    /* nothing fresh within stale_us */
    LBR_VALUE_INVALID,  /* fresh, but the node says unavailable (see reason) */
    LBR_VALUE_OK,
} lbr_value_state_t;

/* Current state of a watched signal; `out` gets the last sample if any. */
lbr_value_state_t lbr_value_table_get(const lbr_value_table_t *t, lbr_signal_id_t signal,
                                      uint64_t now_us, lbr_sample_t *out);
