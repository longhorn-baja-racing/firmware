#include <stdlib.h>

#include "lbr/bytes.h"
#include "lbr/logging.h"
#include "lbr/version.h"
#include "lbr_test.h"

/* In-memory storage that can be told to fail, to model a dying card. */
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;
    int      fail_writes;
    int      syncs;
    int      closed;
} mem_store_t;

static int mem_write(void *ctx, const uint8_t *b, size_t n)
{
    mem_store_t *m = ctx;
    if (m->fail_writes > 0) {
        m->fail_writes--;
        return -1;
    }
    if (m->len + n > m->cap) {
        return -1;
    }
    memcpy(m->buf + m->len, b, n);
    m->len += n;
    return 0;
}

static int mem_sync(void *ctx)
{
    ((mem_store_t *)ctx)->syncs++;
    return 0;
}

static int mem_close(void *ctx)
{
    ((mem_store_t *)ctx)->closed++;
    return 0;
}

static uint8_t s_disk[256 * 1024];
static uint8_t s_queue[4096];
static uint8_t s_chunk[LBR_LOG_MAX_RECORD + 512];

static lbr_log_storage_t storage_for(mem_store_t *m)
{
    memset(m, 0, sizeof *m);
    m->buf              = s_disk;
    m->cap              = sizeof s_disk;
    lbr_log_storage_t s = {mem_write, mem_sync, mem_close, m};
    return s;
}

static void open_logger(lbr_logger_t *lg, mem_store_t *m, size_t queue_cap)
{
    lbr_log_config_t cfg = {
        .queue_mem        = s_queue,
        .queue_cap        = queue_cap,
        .chunk            = s_chunk,
        .chunk_cap        = sizeof s_chunk,
        .sync_interval_ms = 1000,
    };
    CHECK_EQ_I(lbr_log_init(lg, &cfg), 0);
    lbr_log_storage_t st      = storage_for(m);
    lbr_log_session_t session = {
        .fw_version    = LBR_FW_VERSION,
        .node_id       = 1,
        .reset_reason  = LBR_RESET_WATCHDOG,
        .boot_count    = 42,
        .start_time_us = 1000,
        .config_text   = "node=gateway\nnode_id=1\n",
    };
    CHECK_EQ_I(lbr_log_open(lg, &st, &session), 0);
    CHECK(m->syncs == 1); /* session record made durable immediately */
}

static lbr_sample_t make_sample(uint32_t seq)
{
    lbr_sample_t s = {
        .signal    = LBR_SIG_STRAIN_1,
        .source    = 7,
        .type      = LBR_VAL_F32,
        .valid     = true,
        .synced    = true,
        .seq       = seq,
        .t_mono_us = 1000000u + seq * 1000u,
        .v.f32     = (float)seq,
    };
    return s;
}

static void test_write_and_read_back(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);

    for (uint32_t i = 0; i < 50; i++) {
        lbr_sample_t s = make_sample(i);
        CHECK_EQ_I(lbr_log_append_sample(&lg, &s), 0);
        if (i % 10 == 9) {
            lbr_log_pump(&lg, 2000000);
        }
    }
    lbr_health_record_t h = {.node_id = 1, .uptime_ms = 99};
    CHECK_EQ_I(lbr_log_append_health(&lg, &h), 0);
    lbr_can_frame_t f = {.id = 0x123, .dlc = 2, .data = {9, 8}};
    CHECK_EQ_I(lbr_log_append_can_frame(&lg, 555, LBR_CAN_REC_FLAG_EXT, &f), 0);
    CHECK_EQ_I(lbr_log_append_text(&lg, "hello"), 0);
    CHECK_EQ_I(lbr_log_close(&lg, 3000000), 0);
    CHECK_EQ_I(m.closed, 1);

    lbr_log_reader_t r;
    lbr_log_record_t rec;
    lbr_log_reader_init(&r, m.buf, m.len);
    CHECK(lbr_log_reader_next(&r, &rec));
    CHECK_EQ_U(rec.type, LBR_REC_SESSION);
    CHECK_EQ_U(lbr_get_u16(rec.payload), LBR_LOG_FORMAT_VERSION);
    CHECK_EQ_U(lbr_get_u16(rec.payload + 2), LBR_PROTOCOL_VERSION);
    CHECK_EQ_U(rec.payload[17], LBR_RESET_WATCHDOG); /* boot reason observable next session */
    CHECK_EQ_U(lbr_get_u32(rec.payload + 20), 42u);
    CHECK(memcmp(rec.payload + LBR_LOG_SESSION_FIXED, "node=gateway", 12) == 0);

    for (uint32_t i = 0; i < 50; i++) {
        CHECK(lbr_log_reader_next(&r, &rec));
        CHECK_EQ_U(rec.type, LBR_REC_SAMPLE);
        lbr_sample_t s;
        CHECK_EQ_U(lbr_sample_decode(rec.payload, rec.len, &s), LBR_SAMPLE_WIRE_SIZE);
        CHECK_EQ_U(s.seq, i);
    }
    CHECK(lbr_log_reader_next(&r, &rec) && rec.type == LBR_REC_HEALTH);
    CHECK(lbr_log_reader_next(&r, &rec) && rec.type == LBR_REC_CAN_FRAME);
    CHECK(lbr_log_reader_next(&r, &rec) && rec.type == LBR_REC_TEXT);
    CHECK(lbr_log_reader_next(&r, &rec) && rec.type == LBR_REC_END);
    CHECK(!lbr_log_reader_next(&r, &rec));
    CHECK_EQ_U(r.corrupt_regions, 0u);
    CHECK_EQ_U(r.truncated_tail, 0u);
}

static size_t count_records(const uint8_t *buf, size_t len, lbr_log_reader_t *r)
{
    lbr_log_record_t rec;
    size_t           n = 0;
    lbr_log_reader_init(r, buf, len);
    while (lbr_log_reader_next(r, &rec)) {
        n++;
    }
    return n;
}

static void test_truncated_tail_every_offset(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);
    size_t ends[32];
    size_t n_ends  = 0;
    ends[n_ends++] = m.len;
    for (uint32_t i = 0; i < 20; i++) {
        lbr_sample_t s = make_sample(i);
        lbr_log_append_sample(&lg, &s);
        lbr_log_pump(&lg, 0);
        ends[n_ends++] = m.len;
    }

    /* Power loss at every byte: every record fully written before the cut is
     * recovered, nothing after it, and the partial tail is reported. */
    for (size_t cut = 0; cut <= m.len; cut++) {
        size_t complete = 0;
        while (complete < n_ends && ends[complete] <= cut) {
            complete++;
        }
        lbr_log_reader_t r;
        size_t           got = count_records(m.buf, cut, &r);
        CHECK_EQ_U(got, complete);
        size_t last_end = complete ? ends[complete - 1] : 0;
        CHECK_EQ_U(r.truncated_tail + r.skipped_bytes, cut - last_end);
        CHECK_EQ_U(r.corrupt_regions, 0u);
    }
}

static void test_corruption_resync(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);
    for (uint32_t i = 0; i < 10; i++) {
        lbr_sample_t s = make_sample(i);
        lbr_log_append_sample(&lg, &s);
    }
    lbr_log_close(&lg, 0);

    lbr_log_reader_t r;
    size_t           clean = count_records(m.buf, m.len, &r);
    CHECK_EQ_U(clean, 12u); /* session + 10 + end */

    /* Flip a byte inside record #3's payload: only that record is lost. */
    size_t rec_size = LBR_LOG_HEADER_SIZE + LBR_SAMPLE_WIRE_SIZE + LBR_LOG_TRAILER_SIZE;
    size_t first    = m.len - 10 * rec_size - (LBR_LOG_HEADER_SIZE + LBR_LOG_END_SIZE + 4);
    m.buf[first + 3 * rec_size + 10] ^= 0x40;
    CHECK_EQ_U(count_records(m.buf, m.len, &r), clean - 1);
    CHECK_EQ_U(r.corrupt_regions, 1u);
    CHECK_EQ_U(r.skipped_bytes, rec_size);

    /* Garbage spliced into the stream (e.g. a torn FAT cluster). */
    uint8_t *copy = malloc(m.len + 37);
    memcpy(copy, m.buf, first + 5 * rec_size);
    memset(copy + first + 5 * rec_size, 0xA5, 37); /* sync-looking noise */
    memcpy(copy + first + 5 * rec_size + 37, m.buf + first + 5 * rec_size,
           m.len - first - 5 * rec_size);
    CHECK_EQ_U(count_records(copy, m.len + 37, &r), clean - 1);
    CHECK_EQ_U(r.skipped_bytes, rec_size + 37);
    free(copy);
}

static void test_queue_pressure_writes_loss_record(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    /* Tiny queue + SD stall (no pump): the queue fills and drops are counted. */
    open_logger(&lg, &m, LBR_LOG_MAX_RECORD + 64);
    uint32_t accepted = 0;
    uint32_t dropped  = 0;
    for (uint32_t i = 0; i < 300; i++) {
        lbr_sample_t s = make_sample(i);
        if (lbr_log_append_sample(&lg, &s) == 0) {
            accepted++;
        } else {
            dropped++;
        }
    }
    CHECK(dropped > 0);
    lbr_log_stats_t st;
    lbr_log_get_stats(&lg, &st);
    CHECK_EQ_U(st.records_dropped, dropped);
    CHECK(st.queue_high_water > 9000);

    /* Stall ends: queue drains and a LOSS record marks the gap in the file. */
    lbr_log_pump(&lg, 5000000);
    lbr_sample_t after = make_sample(1000);
    CHECK_EQ_I(lbr_log_append_sample(&lg, &after), 0);
    lbr_log_close(&lg, 6000000);

    lbr_log_reader_t r;
    lbr_log_record_t rec;
    lbr_log_reader_init(&r, m.buf, m.len);
    uint32_t samples           = 0;
    uint32_t lost              = 0;
    bool     loss_before_after = false;
    while (lbr_log_reader_next(&r, &rec)) {
        if (rec.type == LBR_REC_SAMPLE) {
            lbr_sample_t s;
            lbr_sample_decode(rec.payload, rec.len, &s);
            if (s.seq == 1000) {
                loss_before_after = lost > 0;
            }
            samples++;
        } else if (rec.type == LBR_REC_LOSS) {
            lost += lbr_get_u32(rec.payload);
        }
    }
    CHECK_EQ_U(samples, accepted + 1);
    CHECK_EQ_U(lost, dropped); /* every drop is accounted for in the log itself */
    CHECK(loss_before_after);
}

static void test_failed_card(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);
    m.fail_writes = 1;
    for (uint32_t i = 0; i < 5; i++) {
        lbr_sample_t s = make_sample(i);
        lbr_log_append_sample(&lg, &s);
    }
    lbr_log_pump(&lg, 0);
    lbr_log_stats_t st;
    lbr_log_get_stats(&lg, &st);
    CHECK_EQ_U(st.write_errors, 1u);
    CHECK_EQ_U(st.records_lost_write, 5u);

    /* Card works again: logging continues, and the file says what was lost. */
    lbr_sample_t s = make_sample(99);
    lbr_log_append_sample(&lg, &s);
    lbr_log_pump(&lg, 0);
    lbr_log_get_stats(&lg, &st);
    CHECK_EQ_U(st.write_errors, 1u);
    lbr_log_reader_t r;
    lbr_log_record_t rec;
    uint32_t         lost = 0;
    lbr_log_reader_init(&r, m.buf, m.len);
    while (lbr_log_reader_next(&r, &rec)) {
        if (rec.type == LBR_REC_LOSS) {
            lost += lbr_get_u32(rec.payload);
        }
    }
    CHECK_EQ_U(lost, 5u);

    /* Open on a dead card fails cleanly. */
    lbr_logger_t      lg2;
    mem_store_t       m2;
    lbr_log_config_t  cfg     = {s_queue, sizeof s_queue, s_chunk, sizeof s_chunk, 1000, 0, 0, 0};
    lbr_log_storage_t st2     = storage_for(&m2);
    m2.fail_writes            = 1;
    lbr_log_session_t session = {.config_text = ""};
    CHECK_EQ_I(lbr_log_init(&lg2, &cfg), 0);
    CHECK(lbr_log_open(&lg2, &st2, &session) != 0);
    CHECK(!lbr_log_is_open(&lg2));
    lbr_sample_t s2 = make_sample(0);
    CHECK(lbr_log_append_sample(&lg2, &s2) != 0);
    lbr_log_get_stats(&lg2, &st);
    CHECK_EQ_U(st.records_rejected_closed, 1u);
}

static void test_counter_rollover_in_records(void)
{
    /* seq and counters are stored modulo 2^32 without saturation. */
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);
    lbr_sample_t a = make_sample(0xFFFFFFFFu);
    lbr_sample_t b = make_sample(0);
    lbr_log_append_sample(&lg, &a);
    lbr_log_append_sample(&lg, &b);
    lbr_log_close(&lg, 0);
    lbr_log_reader_t r;
    lbr_log_record_t rec;
    lbr_log_reader_init(&r, m.buf, m.len);
    lbr_log_reader_next(&r, &rec);
    lbr_sample_t sa, sb;
    lbr_log_reader_next(&r, &rec);
    lbr_sample_decode(rec.payload, rec.len, &sa);
    lbr_log_reader_next(&r, &rec);
    lbr_sample_decode(rec.payload, rec.len, &sb);
    CHECK_EQ_I(lbr_seq_delta(sa.seq, sb.seq), 1);
}

static void test_sync_interval(void)
{
    lbr_logger_t lg;
    mem_store_t  m;
    open_logger(&lg, &m, sizeof s_queue);
    lbr_sample_t s = make_sample(1);
    lbr_log_append_sample(&lg, &s);
    lbr_log_pump(&lg, 1000 + 500000); /* < 1 s since open: no fsync yet */
    CHECK_EQ_I(m.syncs, 1);
    lbr_log_append_sample(&lg, &s);
    lbr_log_pump(&lg, 1000 + 1000000);
    CHECK_EQ_I(m.syncs, 2);
    lbr_log_pump(&lg, 1000 + 5000000); /* nothing new written: no needless fsync */
    CHECK_EQ_I(m.syncs, 2);
}

int main(void)
{
    RUN(test_write_and_read_back);
    RUN(test_truncated_tail_every_offset);
    RUN(test_corruption_resync);
    RUN(test_queue_pressure_writes_loss_record);
    RUN(test_failed_card);
    RUN(test_counter_rollover_in_records);
    RUN(test_sync_interval);
    TEST_MAIN_END();
}
