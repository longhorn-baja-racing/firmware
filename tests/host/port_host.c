#include "port_host.h"

#include "lbr/health.h"
#include "lbr/timebase.h"

/* Host stand-ins for the platform hooks implemented by *_esp.c on target. */

static uint64_t s_fake_us;

void port_host_set_time(uint64_t us)
{
    s_fake_us = us;
}

uint64_t lbr_time_port_mono_us(void)
{
    return s_fake_us;
}

void lbr_time_port_lock(void)
{
}

void lbr_time_port_unlock(void)
{
}

lbr_reset_reason_t lbr_health_reset_reason(void)
{
    return LBR_RESET_POWER_ON;
}

uint32_t lbr_health_boot_count(void)
{
    return 1;
}
