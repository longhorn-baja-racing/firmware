#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/log_sd.h"
#include "lbr/logging.h"

/*
 * SD log service (ESP only): owns the logger, its queue, and a writer task
 * that mounts the card, opens a session, drains the queue, and on card loss
 * closes out and keeps retrying. Used by the gateway (authoritative log) and
 * by local-logging sensor nodes. Producers call lbr_log_append_*() on
 * lbr_log_service_logger() at any time; with no card the appends are
 * counted (records_rejected_closed), never blocking.
 */

typedef struct {
    uint8_t     node_id;
    uint8_t     reset_reason;
    uint32_t    boot_count;
    const char *config_text; /* persisted in every session record */
    size_t      queue_size;  /* bytes */
    uint32_t    sync_interval_ms;

    uint64_t (*now_us)(void); /* shared time base */
    /* Called from the writer task on SD state changes (LBR_EVT_SD_*). */
    void (*on_event)(uint16_t code, bool fault);
} lbr_log_service_config_t;

bool          lbr_log_service_start(const lbr_log_service_config_t *cfg);
lbr_logger_t *lbr_log_service_logger(void);
bool          lbr_log_service_sd_present(void);

/* Name of the open session file ("" if none). */
void lbr_log_service_file_name(char out[LBR_SD_NAME_MAX]);
