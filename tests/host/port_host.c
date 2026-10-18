#include "port_host.h"

#include "lbr/health.h"

/* Host stand-ins for the platform hooks implemented by *_esp.c on target. */

static uint64_t s_fake_us;

void port_host_set_time(uint64_t us)
{
    s_fake_us = us;
}

lbr_reset_reason_t lbr_health_reset_reason(void)
{
    return LBR_RESET_POWER_ON;
}

uint32_t lbr_health_boot_count(void)
{
    return 1;
}
