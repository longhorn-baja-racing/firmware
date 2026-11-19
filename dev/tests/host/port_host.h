#pragma once

#include <stdint.h>

/* A fake local monotonic clock for tests: set it, and hand port_host_mono_us
 * to whatever needs a time source (an lbr_clock_t, a HAL table). */
void     port_host_set_time(uint64_t us);
uint64_t port_host_mono_us(void);
