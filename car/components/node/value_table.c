#include "lbr/value_table.h"

#include <string.h>

void lbr_value_table_init(lbr_value_table_t *t, uint64_t stale_us)
{
    memset(t, 0, sizeof *t);
    t->stale_us = stale_us;
}

bool lbr_value_table_watch(lbr_value_table_t *t, lbr_signal_id_t signal, uint8_t source)
{
    if (t->n >= LBR_VALUE_TABLE_MAX) {
        return false;
    }
    lbr_value_entry_t *e = &t->entries[t->n++];
    memset(e, 0, sizeof *e);
    e->signal = signal;
    e->source = source;
    return true;
}

void lbr_value_table_offer(lbr_value_table_t *t, const lbr_sample_t *s, uint64_t rx_us)
{
    for (size_t i = 0; i < t->n; i++) {
        lbr_value_entry_t *e = &t->entries[i];
        if (e->signal == s->signal && (e->source == 0 || e->source == s->source)) {
            e->last  = *s;
            e->rx_us = rx_us;
            e->have  = true;
        }
    }
}

lbr_value_state_t lbr_value_table_get(const lbr_value_table_t *t, lbr_signal_id_t signal,
                                      uint64_t now_us, lbr_sample_t *out)
{
    for (size_t i = 0; i < t->n; i++) {
        const lbr_value_entry_t *e = &t->entries[i];
        if (e->signal != signal) {
            continue;
        }
        if (!e->have) {
            return LBR_VALUE_NONE;
        }
        if (out != NULL) {
            *out = e->last;
        }
        if (now_us > e->rx_us && now_us - e->rx_us > t->stale_us) {
            return LBR_VALUE_STALE;
        }
        return e->last.valid ? LBR_VALUE_OK : LBR_VALUE_INVALID;
    }
    return LBR_VALUE_NONE;
}
