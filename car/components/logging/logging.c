#include "lbr/logging.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/version.h"

static void lock(lbr_logger_t *lg)
{
    if (lg->cfg.lock != NULL) {
        lg->cfg.lock(lg->cfg.lock_ctx);
    }
}

static void unlock(lbr_logger_t *lg)
{
    if (lg->cfg.unlock != NULL) {
        lg->cfg.unlock(lg->cfg.lock_ctx);
    }
}

/* ------------------------------------------------------------------------ */
/* Framing                                                                   */

size_t lbr_log_frame(uint8_t type, const uint8_t *payload, size_t len, uint8_t *out, size_t cap)
{
    size_t total = LBR_LOG_HEADER_SIZE + len + LBR_LOG_TRAILER_SIZE;
    if (len > LBR_LOG_MAX_PAYLOAD || total > cap) {
        return 0;
    }
    out[0] = LBR_LOG_SYNC0;
    out[1] = LBR_LOG_SYNC1;
    out[2] = type;
    out[3] = 0;
    lbr_put_u16(out + 4, (uint16_t)len);
    if (len > 0) {
        /* memmove: callers may build the payload in place at out + header. */
        memmove(out + LBR_LOG_HEADER_SIZE, payload, len);
    }
    uint32_t crc = lbr_crc32(out + 2, LBR_LOG_HEADER_SIZE - 2 + len);
    lbr_put_u32(out + LBR_LOG_HEADER_SIZE + len, crc);
    return total;
}

size_t lbr_log_encode_session(const lbr_log_session_t *s, uint8_t *buf, size_t cap)
{
    size_t text_len = s->config_text ? strlen(s->config_text) : 0;
    size_t total    = LBR_LOG_SESSION_FIXED + text_len;
    if (total > cap || total > LBR_LOG_MAX_PAYLOAD) {
        return 0;
    }
    lbr_put_u16(buf + 0, LBR_LOG_FORMAT_VERSION);
    lbr_put_u16(buf + 2, LBR_PROTOCOL_VERSION);
    lbr_put_u32(buf + 4, s->fw_version);
    lbr_put_u32(buf + 8, lbr_catalog_hash());
    lbr_put_u32(buf + 12, lbr_crc32(s->config_text ? s->config_text : "", text_len));
    buf[16] = s->node_id;
    buf[17] = s->reset_reason;
    lbr_put_u16(buf + 18, 0);
    lbr_put_u32(buf + 20, s->boot_count);
    lbr_put_u64(buf + 24, s->start_time_us);
    if (text_len > 0) {
        memcpy(buf + LBR_LOG_SESSION_FIXED, s->config_text, text_len);
    }
    return total;
}

/* ------------------------------------------------------------------------ */
/* Logger                                                                    */

int lbr_log_init(lbr_logger_t *lg, const lbr_log_config_t *cfg)
{
    if (cfg->queue_mem == NULL || cfg->queue_cap < LBR_LOG_MAX_RECORD + LBR_RINGBUF_OVERHEAD ||
        cfg->chunk == NULL || cfg->chunk_cap < LBR_LOG_MAX_RECORD) {
        return -1;
    }
    memset(lg, 0, sizeof *lg);
    lg->cfg = *cfg;
    lbr_ringbuf_init(&lg->rb, cfg->queue_mem, cfg->queue_cap);
    return 0;
}

int lbr_log_open(lbr_logger_t *lg, const lbr_log_storage_t *storage,
                 const lbr_log_session_t *session)
{
    /* Build the payload in place inside the staging chunk, then frame it. */
    uint8_t *rec     = lg->cfg.chunk;
    uint8_t *payload = rec + LBR_LOG_HEADER_SIZE;
    size_t   plen    = lbr_log_encode_session(
        session, payload, lg->cfg.chunk_cap - LBR_LOG_HEADER_SIZE - LBR_LOG_TRAILER_SIZE);
    size_t n = plen ? lbr_log_frame(LBR_REC_SESSION, payload, plen, rec, lg->cfg.chunk_cap) : 0;
    if (n == 0) {
        return -1;
    }
    if (storage->write(storage->ctx, rec, n) != 0 || storage->sync(storage->ctx) != 0) {
        lg->stats.write_errors++;
        return -1;
    }

    lock(lg);
    lg->st                  = *storage;
    lg->open                = true;
    lg->dirty               = false;
    lg->last_sync_us        = session->start_time_us;
    lg->stats.last_write_us = session->start_time_us;
    lg->stats.records_written++;
    unlock(lg);
    return 0;
}

/* Called with the lock held. */
static bool flush_pending_loss(lbr_logger_t *lg, uint64_t t_us, size_t need_after)
{
    if (lg->pending_loss_records == 0) {
        return true;
    }
    uint8_t payload[LBR_LOG_LOSS_SIZE];
    uint8_t rec[LBR_LOG_HEADER_SIZE + LBR_LOG_LOSS_SIZE + LBR_LOG_TRAILER_SIZE];
    lbr_put_u32(payload, lg->pending_loss_records);
    lbr_put_u32(payload + 4, lg->pending_loss_bytes);
    lbr_put_u64(payload + 8, t_us);
    size_t n = lbr_log_frame(LBR_REC_LOSS, payload, sizeof payload, rec, sizeof rec);
    if (n + LBR_RINGBUF_OVERHEAD + need_after > lbr_ringbuf_free(&lg->rb) ||
        !lbr_ringbuf_push(&lg->rb, rec, n)) {
        return false;
    }
    lg->pending_loss_records = 0;
    lg->pending_loss_bytes   = 0;
    lg->stats.records_queued++;
    return true;
}

static int enqueue(lbr_logger_t *lg, uint8_t type, const uint8_t *payload, size_t len,
                   uint64_t t_us)
{
    if (len > LBR_LOG_MAX_PAYLOAD) {
        return -1;
    }
    /* Header and CRC built here; payload copied straight into the queue. */
    uint8_t hdr[LBR_LOG_HEADER_SIZE] = {LBR_LOG_SYNC0, LBR_LOG_SYNC1, type, 0};
    uint8_t trl[LBR_LOG_TRAILER_SIZE];
    lbr_put_u16(hdr + 4, (uint16_t)len);
    lbr_put_u32(trl, lbr_crc32_update(lbr_crc32(hdr + 2, LBR_LOG_HEADER_SIZE - 2), payload, len));
    const lbr_iov_t iov[3] = {{hdr, sizeof hdr}, {payload, len}, {trl, sizeof trl}};
    size_t          n      = sizeof hdr + len + sizeof trl;

    lock(lg);
    int rc = -1;
    if (!lg->open) {
        /* No session (e.g. no card): the caller owns reporting this state. */
        lg->stats.records_rejected_closed++;
    } else if (flush_pending_loss(lg, t_us, n + LBR_RINGBUF_OVERHEAD) &&
               lbr_ringbuf_pushv(&lg->rb, iov, 3)) {
        lg->stats.records_queued++;
        rc = 0;
    } else {
        lg->pending_loss_records++;
        lg->pending_loss_bytes += (uint32_t)n;
        lg->stats.records_dropped++;
        lg->stats.bytes_dropped += (uint32_t)n;
    }
    lbr_high_water_update(&lg->stats.queue_high_water, lg->rb.high_water, lg->rb.cap);
    unlock(lg);
    return rc;
}

int lbr_log_append_raw(lbr_logger_t *lg, uint8_t type, const uint8_t *payload, size_t len)
{
    return enqueue(lg, type, payload, len, 0);
}

int lbr_log_append_sample(lbr_logger_t *lg, const lbr_sample_t *s)
{
    uint8_t buf[LBR_SAMPLE_WIRE_SIZE];
    if (lbr_sample_encode(s, buf, sizeof buf) == 0) {
        return -1;
    }
    return enqueue(lg, LBR_REC_SAMPLE, buf, sizeof buf, s->t_mono_us);
}

int lbr_log_append_health(lbr_logger_t *lg, const lbr_health_record_t *h)
{
    uint8_t buf[LBR_HEALTH_WIRE_SIZE];
    if (lbr_health_encode(h, buf, sizeof buf) == 0) {
        return -1;
    }
    return enqueue(lg, LBR_REC_HEALTH, buf, sizeof buf, 0);
}

int lbr_log_append_can_frame(lbr_logger_t *lg, uint64_t t_us, uint8_t flags,
                             const lbr_can_frame_t *f)
{
    uint8_t buf[LBR_CAN_FRAME_REC_SIZE];
    if (lbr_can_frame_rec_encode(t_us, flags, f, buf, sizeof buf) == 0) {
        return -1;
    }
    return enqueue(lg, LBR_REC_CAN_FRAME, buf, sizeof buf, t_us);
}

int lbr_log_append_text(lbr_logger_t *lg, const char *text)
{
    size_t len = strlen(text);
    if (len > LBR_LOG_MAX_PAYLOAD) {
        len = LBR_LOG_MAX_PAYLOAD;
    }
    return enqueue(lg, LBR_REC_TEXT, (const uint8_t *)text, len, 0);
}

static int write_chunk(lbr_logger_t *lg, size_t used, uint32_t records, uint64_t now_us)
{
    if (used == 0) {
        return 0;
    }
    if (lg->st.write(lg->st.ctx, lg->cfg.chunk, used) != 0) {
        lock(lg);
        lg->stats.write_errors++;
        lg->stats.records_lost_write += records;
        /* The next record that lands (this session or the next one opened
         * by the same logger) is preceded by a LOSS record for these. */
        lg->pending_loss_records += records;
        lg->pending_loss_bytes += (uint32_t)used;
        unlock(lg);
        return -1;
    }
    lg->stats.records_written += records;
    lg->stats.last_write_us = now_us;
    lg->dirty               = true;
    return 0;
}

int lbr_log_pump(lbr_logger_t *lg, uint64_t now_us)
{
    if (!lg->open) {
        return 0;
    }
    int      total   = 0;
    size_t   used    = 0;
    uint32_t records = 0;

    for (;;) {
        lock(lg);
        size_t next = lbr_ringbuf_peek_len(&lg->rb);
        if (next == 0) {
            /* Queue drained: close any open loss gap now that there is room. */
            flush_pending_loss(lg, now_us, 0);
            next = lbr_ringbuf_peek_len(&lg->rb);
        }
        if (next == 0 || used + next > lg->cfg.chunk_cap) {
            unlock(lg);
            if (next == 0) {
                break;
            }
            write_chunk(lg, used, records, now_us);
            total += (int)records;
            used    = 0;
            records = 0;
            continue;
        }
        used += lbr_ringbuf_pop(&lg->rb, lg->cfg.chunk + used, lg->cfg.chunk_cap - used);
        records++;
        unlock(lg);
    }
    write_chunk(lg, used, records, now_us);
    total += (int)records;

    if (lg->dirty && now_us - lg->last_sync_us >= (uint64_t)lg->cfg.sync_interval_ms * 1000u) {
        if (lg->st.sync(lg->st.ctx) != 0) {
            lg->stats.sync_errors++;
        }
        lg->dirty        = false;
        lg->last_sync_us = now_us;
    }
    return total;
}

int lbr_log_close(lbr_logger_t *lg, uint64_t now_us)
{
    if (!lg->open) {
        return -1;
    }
    lbr_log_pump(lg, now_us);

    uint8_t payload[LBR_LOG_END_SIZE];
    lbr_put_u64(payload, now_us);
    lbr_put_u32(payload + 8, lg->stats.records_written + 1);
    size_t n =
        lbr_log_frame(LBR_REC_END, payload, sizeof payload, lg->cfg.chunk, lg->cfg.chunk_cap);
    int rc = write_chunk(lg, n, 1, now_us);
    if (lg->st.sync(lg->st.ctx) != 0) {
        lg->stats.sync_errors++;
        rc = -1;
    }
    if (lg->st.close(lg->st.ctx) != 0) {
        rc = -1;
    }
    lock(lg);
    lg->open = false;
    unlock(lg);
    return rc;
}

void lbr_log_get_stats(lbr_logger_t *lg, lbr_log_stats_t *out)
{
    lock(lg);
    *out = lg->stats;
    unlock(lg);
}

/* ------------------------------------------------------------------------ */
/* Reader                                                                    */

void lbr_log_reader_init(lbr_log_reader_t *r, const uint8_t *buf, size_t len)
{
    memset(r, 0, sizeof *r);
    r->buf = buf;
    r->len = len;
}

/* A complete, CRC-valid record starting at `pos`. */
static bool record_at(const lbr_log_reader_t *r, size_t pos, lbr_log_record_t *out, size_t *total)
{
    size_t         left = r->len - pos;
    const uint8_t *p    = r->buf + pos;
    if (left < LBR_LOG_HEADER_SIZE + LBR_LOG_TRAILER_SIZE || p[0] != LBR_LOG_SYNC0 ||
        p[1] != LBR_LOG_SYNC1 || p[3] != 0) {
        return false;
    }
    size_t plen = lbr_get_u16(p + 4);
    size_t n    = LBR_LOG_HEADER_SIZE + plen + LBR_LOG_TRAILER_SIZE;
    if (plen > LBR_LOG_MAX_PAYLOAD || left < n ||
        lbr_get_u32(p + LBR_LOG_HEADER_SIZE + plen) !=
            lbr_crc32(p + 2, LBR_LOG_HEADER_SIZE - 2 + plen)) {
        return false;
    }
    out->type    = p[2];
    out->payload = p + LBR_LOG_HEADER_SIZE;
    out->len     = plen;
    out->offset  = pos;
    *total       = n;
    return true;
}

/* The bytes from `pos` to the end could be the start of a record that was
 * cut off (power loss mid-write). */
static bool tail_candidate(const lbr_log_reader_t *r, size_t pos)
{
    size_t         left = r->len - pos;
    const uint8_t *p    = r->buf + pos;
    if (p[0] != LBR_LOG_SYNC0 || (left >= 2 && p[1] != LBR_LOG_SYNC1)) {
        return false;
    }
    if (left < LBR_LOG_HEADER_SIZE) {
        return true;
    }
    size_t plen = lbr_get_u16(p + 4);
    return plen <= LBR_LOG_MAX_PAYLOAD && left < LBR_LOG_HEADER_SIZE + plen + LBR_LOG_TRAILER_SIZE;
}

static bool any_record_after(const lbr_log_reader_t *r, size_t pos)
{
    lbr_log_record_t tmp;
    size_t           n;
    for (size_t q = pos + 1; q < r->len; q++) {
        if (record_at(r, q, &tmp, &n)) {
            return true;
        }
    }
    return false;
}

bool lbr_log_reader_next(lbr_log_reader_t *r, lbr_log_record_t *out)
{
    bool in_garbage = false;
    while (r->pos < r->len) {
        size_t total;
        if (record_at(r, r->pos, out, &total)) {
            r->pos += total;
            r->records++;
            return true;
        }
        if (tail_candidate(r, r->pos) && !any_record_after(r, r->pos)) {
            r->truncated_tail = r->len - r->pos;
            r->pos            = r->len;
            return false;
        }
        if (!in_garbage) {
            r->corrupt_regions++;
            in_garbage = true;
        }
        r->skipped_bytes++;
        r->pos++;
    }
    return false;
}
