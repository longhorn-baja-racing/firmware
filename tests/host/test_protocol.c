#include <stdint.h>

#include "lbr/protocol.h"
#include "lbr/timebase.h"

/*
 * Placeholder host test — proves the shared headers compile/link on the host
 * and gives CI something to run. Real tests must cover encode/decode
 * round-trips, timestamp/sequence rollover, time-base sync, queue pressure,
 * and log recovery (see README "Test workflow").
 */
int main(void)
{
    lbr_sample_t s = {
        .signal    = LBR_SIG_ENGINE_RPM,
        .t_mono_us = 123,
        .seq       = 1,
        .type      = LBR_VAL_U32,
        .valid     = true,
        .v.u32     = 3600,
    };

    uint8_t buf[64];

    /* Encode is a stub today (returns 0); exercise the contract shape. */
    size_t n = lbr_sample_encode(&s, buf, sizeof buf);

    /* Time base starts un-synced. */
    if (lbr_time_is_synced()) {
        return 1;
    }

    (void)n;
    return 0;
}
