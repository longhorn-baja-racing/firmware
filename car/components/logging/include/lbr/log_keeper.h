#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/logging.h"

/*
 * Keeps a logger attached to its card: the decisions of the SD log service,
 * independent of the platform. Every pump period it either writes queued
 * records, or (no session open) tries to mount and start a new session file
 * at most once per retry interval. A write error closes the session out and
 * the card is remounted on a later pass, so a pulled or failing card costs a
 * gap that is counted, never the logger itself.
 *
 * Events: SD_MISSING (fault, once per outage), SD_MOUNTED, SD_WRITE_ERROR
 * (fault). log_service_esp.c drives it on target; the simulator drives it
 * with a simulated card.
 */

#define LBR_LOG_KEEPER_NAME_MAX 16 /* matches LBR_SD_NAME_MAX */
#define LBR_LOG_KEEPER_RETRY_US 5000000u

typedef struct {
    void *ctx;
    bool (*mount)(void *ctx);
    void (*unmount)(void *ctx);
    /* Create the next session file; fills the storage backend and its name. */
    bool (*open_file)(void *ctx, lbr_log_storage_t *st, char name[LBR_LOG_KEEPER_NAME_MAX]);
    void (*on_event)(void *ctx, uint16_t code, bool fault);
} lbr_log_keeper_hal_t;

typedef struct {
    lbr_log_keeper_hal_t hal;
    lbr_logger_t        *log;
    lbr_log_session_t    session; /* start_time_us is set at each open */
    uint64_t             next_mount_us;
    uint32_t             last_errors;
    bool                 reported;
    bool                 sd_present;
    char                 name[LBR_LOG_KEEPER_NAME_MAX]; /* current file, "" if none */
} lbr_log_keeper_t;

void lbr_log_keeper_init(lbr_log_keeper_t *k, lbr_logger_t *log, const lbr_log_keeper_hal_t *hal,
                         const lbr_log_session_t *session);

/* One pump period. `mono_us` paces mount retries; `now_us` (shared time
 * base) stamps records. */
void lbr_log_keeper_step(lbr_log_keeper_t *k, uint64_t mono_us, uint64_t now_us);

/* Clean shutdown before a reboot: close the session (END record). */
void lbr_log_keeper_close(lbr_log_keeper_t *k, uint64_t now_us);
