#pragma once

#include "lbr/gw_core.h"
#include "lbr/replay.h"

/*
 * Internal to the gateway core (gw_core.c, gw_health.c, gw_links.c): the
 * helpers its parts share. Apps use lbr/gw_core.h.
 */

#define NODE_ID LBR_NODE_CENTRAL

/* A line in the Central's log (hal.log). */
void gw_say(lbr_gw_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* Take / release one of the core's locks (hal.lock). */
void gw_lock(lbr_gw_core_t *c, lbr_gw_lock_t which);
void gw_unlock(lbr_gw_core_t *c, lbr_gw_lock_t which);

/* Transmit one of the Central's own frames and record it like any other. */
void gw_tx(lbr_gw_core_t *c, const lbr_can_frame_t *f);

/* Log a TICK record (lbr/replay.h); call with the STATE lock held. */
void gw_log_tick(lbr_gw_core_t *c, lbr_tick_kind_t kind, uint64_t t_us);
