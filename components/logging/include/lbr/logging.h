#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/health.h"
#include "lbr/protocol.h"

/*
 * Append-oriented, versioned, recoverable log.
 *
 * On the gateway this is the AUTHORITATIVE full-fidelity record; on select
 * high-rate/critical nodes it is local redundancy. Writes go through a bounded
 * queue: a slow or absent live link must NEVER reduce what is written here.
 * Interrupted sessions must be recoverable (power-loss / truncated tail).
 */

typedef struct lbr_logger lbr_logger_t;

typedef struct {
    uint32_t    format_version;
    uint32_t    fw_version;
    uint32_t    protocol_version;
    uint32_t    node_id;
    const char *session_config; /* exact validated config + node topology (see spec) */
} lbr_log_session_t;

/* Open a session; persists session metadata as the first record. */
int lbr_log_open(lbr_logger_t *lg, const lbr_log_session_t *session);

int lbr_log_append_sample(lbr_logger_t *lg, const lbr_sample_t *s);
int lbr_log_append_health(lbr_logger_t *lg, const lbr_health_record_t *h);

int lbr_log_flush(lbr_logger_t *lg);
int lbr_log_close(lbr_logger_t *lg);
