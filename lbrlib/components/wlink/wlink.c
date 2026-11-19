#include "lbr/wlink.h"

#include "lbr/protocol.h"

/* Pure cockpit-link helpers; see lbr/wlink.h. */

/* ------------------------------------------------------------------------ */
/* Datagram sequence                                                         */

#define SEQ_WINDOW 32u

static uint32_t missing(uint32_t bits, unsigned n)
{
    uint32_t m = 0;
    for (unsigned i = 0; i < n; i++) {
        m += ((bits >> i) & 1u) ? 0u : 1u;
    }
    return m;
}

void lbr_seq_window_restart(lbr_seq_window_t *w, uint32_t *gaps)
{
    if (w->have) {
        *gaps += missing(w->bits, SEQ_WINDOW);
    }
    w->have = false;
}

lbr_seq_result_t lbr_seq_window_take(lbr_seq_window_t *w, uint32_t seq, uint32_t *gaps)
{
    if (!w->have) {
        /* Nothing before the first datagram can be missing. */
        *w = (lbr_seq_window_t){.have = true, .last = seq, .bits = 0xFFFFFFFFu};
        return LBR_SEQ_NEW;
    }
    int32_t d = lbr_seq_delta(w->last, seq);
    if (d > 0) {
        if ((uint32_t)d >= SEQ_WINDOW) {
            *gaps += missing(w->bits, SEQ_WINDOW) + (uint32_t)d - SEQ_WINDOW;
            w->bits = 0;
        } else {
            *gaps += missing(w->bits >> (SEQ_WINDOW - (uint32_t)d), (unsigned)d);
            w->bits <<= d;
        }
        w->bits |= 1u;
        w->last = seq;
        return LBR_SEQ_NEW;
    }
    uint32_t back = (uint32_t)(-d);
    if (back >= SEQ_WINDOW) {
        return LBR_SEQ_OLD;
    }
    if (w->bits & (1u << back)) {
        return LBR_SEQ_DUPLICATE;
    }
    w->bits |= 1u << back;
    return LBR_SEQ_LATE;
}
