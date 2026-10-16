#include "lbr/timebase.h"

/*
 * Stub time base. `lbr_time_mono_us` must be backed by a hardware monotonic
 * clock (e.g. esp_timer_get_time) on the node build; host tests can inject a
 * fake. Offset disciplining is a placeholder - replace with the agreed filter.
 */

static int64_t s_offset_us = 0;
static bool    s_synced    = false;

uint64_t lbr_time_mono_us(void)
{
    return 0; /* TODO: back with esp_timer on hardware */
}

void lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us)
{
    s_offset_us = (int64_t)gateway_us - (int64_t)local_rx_us;
    s_synced    = true;
}

uint64_t lbr_time_now_us(void)
{
    return (uint64_t)((int64_t)lbr_time_mono_us() + s_offset_us);
}

bool lbr_time_is_synced(void)
{
    return s_synced;
}
