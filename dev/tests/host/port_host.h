#pragma once

#include <stdint.h>

/* Set the fake local monotonic clock seen by lbr_time_mono_us(). */
void port_host_set_time(uint64_t us);
