#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Shared, CAN-distributed monotonic time base.
 *
 * The gateway broadcasts a time-sync frame; every node disciplines its local
 * clock to it so all logs (gateway and any local node logs) share one timeline.
 * Epoch: gateway boot. Unit: microseconds. See docs/protocol.md.
 */

/* Raw local monotonic microseconds (e.g. esp_timer on hardware). */
uint64_t lbr_time_mono_us(void);

/* Discipline the local clock to a received sync frame: `gateway_us` is the
 * timestamp carried by the frame, `local_rx_us` the local time at reception. */
void lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us);

/* Disciplined shared time base — use this to stamp samples. */
uint64_t lbr_time_now_us(void);

/* True once at least one sync frame has disciplined this node. */
bool lbr_time_is_synced(void);
