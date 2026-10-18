#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/health.h"
#include "lbr/protocol.h"
#include "lbr/ringbuf.h"

/*
 * Append-oriented, versioned, recoverable log.
 *
 * On the gateway this is the AUTHORITATIVE full-fidelity record; on select
 * high-rate/critical nodes it is local redundancy. Producers append through a
 * bounded in-memory queue and never block; a single writer drains it to
 * storage. A full queue drops the record, counts it, and writes a LOSS record
 * into the log as soon as space returns, so every gap is visible in the file.
 *
 * File format (all little-endian):
 *
 *   log    := record*
 *   record := sync[2] type[1] flags[1] len[2] payload[len] crc32[4]
 *
 *   sync   = 0xA5 0x5A
 *   flags  = 0 (reserved)
 *   crc32  = lbr_crc32 over type, flags, len, payload
 *
 * The first record is always SESSION. A reader that hits a bad CRC or a
 * truncated tail skips forward to the next sync and keeps going, so a log cut
 * by power loss loses at most the record being written.
 */

#define LBR_LOG_FORMAT_VERSION 1u
#define LBR_LOG_SYNC0          0xA5u
#define LBR_LOG_SYNC1          0x5Au
#define LBR_LOG_HEADER_SIZE    6u
#define LBR_LOG_TRAILER_SIZE   4u
#define LBR_LOG_MAX_PAYLOAD    2048u
#define LBR_LOG_MAX_RECORD     (LBR_LOG_HEADER_SIZE + LBR_LOG_MAX_PAYLOAD + LBR_LOG_TRAILER_SIZE)

/*
 * SESSION payload:
 *
 *   0  2  format_version      LBR_LOG_FORMAT_VERSION
 *   2  2  protocol_version    LBR_PROTOCOL_VERSION
 *   4  4  fw_version          LBR_FW_VERSION
 *   8  4  catalog_hash
 *   12 4  config_crc          CRC-32 of the config text below
 *   16 1  node_id
 *   17 1  reset_reason        lbr_reset_reason_t of the boot that opened the log
 *   18 2  reserved (0)
 *   20 4  boot_count
 *   24 8  start_time_us       shared time base at open
 *   32 .. config text         exact validated configuration + topology,
 *                             ASCII "key=value\n" lines
 */
#define LBR_LOG_SESSION_FIXED 32u

typedef struct {
    uint32_t    fw_version;
    uint8_t     node_id;
    uint8_t     reset_reason;
    uint32_t    boot_count;
    uint64_t    start_time_us;
    const char *config_text; /* exact validated config + node topology */
} lbr_log_session_t;

/* LOSS payload: dropped_records u32, dropped_bytes u32, t_us u64 (when the
 * gap was closed). END payload: t_us u64, records_written u32. */
#define LBR_LOG_LOSS_SIZE 16u
#define LBR_LOG_END_SIZE  12u

/* ------------------------------------------------------------------------ */
/* Storage backend                                                           */

typedef struct {
    /* Append `len` bytes. Return 0 on success, <0 on error. */
    int (*write)(void *ctx, const uint8_t *buf, size_t len);
    /* Make written data durable (fsync). */
    int (*sync)(void *ctx);
    int (*close)(void *ctx);
    void *ctx;
} lbr_log_storage_t;

/* ------------------------------------------------------------------------ */
/* Logger                                                                    */

typedef struct {
    uint8_t *queue_mem; /* bounded queue storage (caller-owned) */
    size_t   queue_cap;
    uint8_t *chunk; /* write staging buffer, >= LBR_LOG_MAX_RECORD */
    size_t   chunk_cap;
    uint32_t sync_interval_ms; /* fsync at least this often while records flow */

    /* Optional producer/writer lock (NULL on single-threaded hosts). */
    void (*lock)(void *lock_ctx);
    void (*unlock)(void *lock_ctx);
    void *lock_ctx;
} lbr_log_config_t;

typedef struct {
    uint32_t records_queued;
    uint32_t records_written;
    uint32_t records_dropped; /* queue full: never silent, see LOSS records */
    uint32_t bytes_dropped;
    uint32_t records_lost_write;      /* written to storage but storage failed */
    uint32_t records_rejected_closed; /* appended while no session was open */
    uint32_t write_errors;
    uint32_t sync_errors;
    uint32_t queue_high_water; /* percent * 100 */
    uint64_t last_write_us;    /* time of last successful storage write, 0 = never */
} lbr_log_stats_t;

/* Fields are private; exposed so the logger can be statically allocated. */
typedef struct {
    lbr_log_config_t  cfg;
    lbr_ringbuf_t     rb;
    lbr_log_storage_t st;
    bool              open;
    uint32_t          pending_loss_records;
    uint32_t          pending_loss_bytes;
    uint64_t          last_sync_us;
    bool              dirty;
    lbr_log_stats_t   stats;
} lbr_logger_t;

int lbr_log_init(lbr_logger_t *lg, const lbr_log_config_t *cfg);

/* Open a session on `storage`: writes and syncs the SESSION record
 * synchronously so metadata is durable before any data. */
int lbr_log_open(lbr_logger_t *lg, const lbr_log_storage_t *storage,
                 const lbr_log_session_t *session);

/* Producers: non-blocking. Return 0 if queued, <0 if dropped (counted). */
int lbr_log_append_sample(lbr_logger_t *lg, const lbr_sample_t *s);
int lbr_log_append_health(lbr_logger_t *lg, const lbr_health_record_t *h);
int lbr_log_append_can_frame(lbr_logger_t *lg, uint64_t t_us, uint8_t flags,
                             const lbr_can_frame_t *f);
int lbr_log_append_text(lbr_logger_t *lg, const char *text);
int lbr_log_append_raw(lbr_logger_t *lg, uint8_t type, const uint8_t *payload, size_t len);

/* Writer: drain the queue to storage; fsync when `sync_interval_ms` elapsed.
 * Returns the number of records written. */
int lbr_log_pump(lbr_logger_t *lg, uint64_t now_us);

/* Drain everything, write END, sync, close storage. */
int lbr_log_close(lbr_logger_t *lg, uint64_t now_us);

void lbr_log_get_stats(lbr_logger_t *lg, lbr_log_stats_t *out);

static inline bool lbr_log_is_open(const lbr_logger_t *lg)
{
    return lg->open;
}

/* ------------------------------------------------------------------------ */
/* Record framing + recovery reader (shared with host tools)                  */

/* Frame one record into `out`. Returns total bytes, 0 if it does not fit. */
size_t lbr_log_frame(uint8_t type, const uint8_t *payload, size_t len, uint8_t *out, size_t cap);

size_t lbr_log_encode_session(const lbr_log_session_t *s, uint8_t *buf, size_t cap);

typedef struct {
    uint8_t        type;
    const uint8_t *payload; /* points into the reader's buffer */
    size_t         len;
    size_t         offset; /* file offset of the record's sync bytes */
} lbr_log_record_t;

typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;
    uint32_t       records;
    uint32_t       corrupt_regions; /* bad CRC / garbage regions skipped */
    size_t         skipped_bytes;
    size_t         truncated_tail; /* bytes of an incomplete final record */
} lbr_log_reader_t;

void lbr_log_reader_init(lbr_log_reader_t *r, const uint8_t *buf, size_t len);

/* Next valid record. Returns false at end of data. */
bool lbr_log_reader_next(lbr_log_reader_t *r, lbr_log_record_t *out);
