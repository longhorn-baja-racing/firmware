#include "port_host.h"

/* Host stand-ins for platform hooks. The pure layers take their platform
 * through HAL tables; only the fake clock below is shared by several tests. */

static uint64_t s_fake_us;

void port_host_set_time(uint64_t us)
{
    s_fake_us = us;
}

uint64_t port_host_mono_us(void)
{
    return s_fake_us;
}
