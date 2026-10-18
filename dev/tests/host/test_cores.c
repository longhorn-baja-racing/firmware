/*
 * The platform-independent node and gateway cores, driven through fake HALs:
 * the same logic the ESP32 glue and the simulator run.
 */
#include <stdlib.h>

#include "lbr/gw_core.h"
#include "lbr/node_core.h"
#include "lbr/nodes.h"
#include "lbr/version.h"
#include "lbr_test.h"

/* ------------------------------------------------------------------------ */
/* Fake platform                                                             */

#define MAX_TX 4096

typedef struct {
    uint64_t         mono;
    lbr_time_servo_t servo;
    lbr_can_stats_t  can;
    bool             sd;
    bool             lora_ready;
    int              locks_held;
    int              sync_arms;
    lbr_can_frame_t  tx[MAX_TX];
    size_t           n_tx;
    char             last_log[192];
} fake_t;

static uint64_t f_mono(void *ctx)
{
    return ((fake_t *)ctx)->mono;
}

static uint64_t f_now(void *ctx)
{
    fake_t *f = ctx;
    return lbr_time_servo_now(&f->servo, f->mono);
}

static uint64_t f_l2s(void *ctx, uint64_t local)
{
    return lbr_time_servo_map(&((fake_t *)ctx)->servo, local);
}

static bool f_synced(void *ctx)
{
    return ((fake_t *)ctx)->servo.synced;
}

static lbr_time_update_t f_apply(void *ctx, uint64_t gw, uint64_t local, uint16_t epoch)
{
    return lbr_time_servo_update(&((fake_t *)ctx)->servo, gw, local, epoch);
}

static void f_servo(void *ctx, lbr_time_servo_t *out)
{
    *out = ((fake_t *)ctx)->servo;
}

static bool f_send(void *ctx, const lbr_can_frame_t *fr)
{
    fake_t *f = ctx;
    if (!f->can.running || f->n_tx >= MAX_TX) {
        f->can.tx_dropped++;
        return false;
    }
    f->tx[f->n_tx++] = *fr;
    f->can.tx_frames++;
    return true;
}

static void f_stats(void *ctx, lbr_can_stats_t *out)
{
    *out = ((fake_t *)ctx)->can;
}

static bool f_sd(void *ctx)
{
    return ((fake_t *)ctx)->sd;
}

static bool f_lora_ready(void *ctx)
{
    return ((fake_t *)ctx)->lora_ready;
}

static void f_lock(void *ctx)
{
    ((fake_t *)ctx)->locks_held++;
}

static void f_unlock(void *ctx)
{
    ((fake_t *)ctx)->locks_held--;
}

static void f_gw_lock(void *ctx, lbr_gw_lock_t which)
{
    (void)which;
    f_lock(ctx);
}

static void f_gw_unlock(void *ctx, lbr_gw_lock_t which)
{
    (void)which;
    f_unlock(ctx);
}

static void f_sync_arm(void *ctx)
{
    ((fake_t *)ctx)->sync_arms++;
}

static void f_log(void *ctx, lbr_log_level_t level, const char *text)
{
    (void)level;
    snprintf(((fake_t *)ctx)->last_log, sizeof((fake_t *)ctx)->last_log, "%s", text);
}

static void fake_init(fake_t *f)
{
    memset(f, 0, sizeof *f);
    lbr_time_servo_init(&f->servo);
    f->can.running = true;
    f->mono        = 1000000;
}

static lbr_node_hal_t node_hal(fake_t *f)
{
    return (lbr_node_hal_t){
        .ctx             = f,
        .mono_us         = f_mono,
        .now_us          = f_now,
        .local_to_shared = f_l2s,
        .time_synced     = f_synced,
        .time_apply      = f_apply,
        .time_servo      = f_servo,
        .can_send        = f_send,
        .can_stats       = f_stats,
        .sd_present      = f_sd,
        .lock            = f_lock,
        .unlock          = f_unlock,
        .log             = f_log,
    };
}

/* Count frames carrying EVENT/FAULT `code` since index `from`. */
static int count_events(const fake_t *f, size_t from, lbr_signal_id_t sig, lbr_event_code_t code)
{
    int n = 0;
    for (size_t i = from; i < f->n_tx; i++) {
        lbr_sample_t s;
        if (lbr_can_decode_sample(&f->tx[i], 0, &s) && s.signal == sig &&
            (s.v.u32 & 0xFFFFu) == code) {
            n++;
        }
    }
    return n;
}

/* A controllable sensor driver. */
typedef struct {
    int   fail_init;
    int   fail_read;
    float value;
    int   inits;
} fake_sensor_t;

static int fs_init(lbr_sensor_t *self)
{
    fake_sensor_t *s = self->ctx;
    s->inits++;
    if (s->fail_init > 0) {
        s->fail_init--;
        return -1;
    }
    return 0;
}

static int fs_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us;
    fake_sensor_t *s = self->ctx;
    if (s->fail_read > 0) {
        s->fail_read--;
        return -1;
    }
    for (size_t i = 0; i < self->n_signals; i++) {
        out[i] = (lbr_reading_t){.valid = true, .v.f32 = s->value + (float)i};
    }
    return 0;
}

static const lbr_signal_id_t k_sigs[] = {LBR_SIG_SUSP_TRAVEL_FL, LBR_SIG_SUSP_TRAVEL_FR};

typedef struct {
    fake_t            fake;
    fake_sensor_t     fs;
    lbr_sensor_t      sensor;
    lbr_acq_t         acq;
    lbr_node_config_t cfg;
    lbr_node_core_t   core;
} node_rig_t;

static bool rig_init(node_rig_t *r, uint16_t rate_hz)
{
    memset(r, 0, sizeof *r);
    fake_init(&r->fake);
    r->sensor = (lbr_sensor_t){
        .name      = "travel",
        .signals   = k_sigs,
        .n_signals = 2,
        .init      = fs_init,
        .read      = fs_read,
        .ctx       = &r->fs,
    };
    r->acq = (lbr_acq_t){.sensor = &r->sensor, .rate_hz = rate_hz, .publish_div = 1};
    r->cfg = (lbr_node_config_t){
        .node_id          = LBR_NODE_SUSPENSION,
        .acq              = &r->acq,
        .n_acq            = 1,
        .health_period_ms = 1000,
    };
    lbr_node_hal_t  hal  = node_hal(&r->fake);
    lbr_node_boot_t boot = {
        .reset_reason        = LBR_RESET_POWER_ON,
        .boot_count          = 7,
        .can_bitrate         = 1000000,
        .can_budget_permille = 150,
    };
    return lbr_node_core_init(&r->core, &r->cfg, &hal, &boot);
}

/* Feed the node a gateway TIME_SYNC + FOLLOW_UP pair. */
static void feed_sync(node_rig_t *r, uint8_t seq, uint16_t epoch, uint64_t gw_us)
{
    lbr_can_frame_t f;
    lbr_can_encode_time_sync(LBR_NODE_GATEWAY, seq, epoch, &f);
    lbr_node_core_on_frame(&r->core, &f, r->fake.mono);
    lbr_can_encode_time_follow_up(LBR_NODE_GATEWAY, seq, gw_us, &f);
    lbr_node_core_on_frame(&r->core, &f, r->fake.mono + 50);
}

/* ------------------------------------------------------------------------ */
/* Node core                                                                 */

static void test_node_boot_and_acquire(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    CHECK(strstr(r.fake.last_log, "suspension") != NULL);
    lbr_node_core_start(&r.core, true, false, NULL);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_EVENT, LBR_EVT_BOOT), 1);

    lbr_node_core_slot_start(&r.core, 0);
    r.fs.value  = 12.5f;
    size_t from = r.fake.n_tx;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    CHECK_EQ_U(r.fake.n_tx - from, 2);
    lbr_sample_t s;
    CHECK(lbr_can_decode_sample(&r.fake.tx[from], 0, &s));
    CHECK_EQ_U(s.signal, LBR_SIG_SUSP_TRAVEL_FL);
    CHECK_EQ_U(s.source, LBR_NODE_SUSPENSION);
    CHECK(s.valid && s.v.f32 == 12.5f);
    CHECK(!s.synced); /* no gateway yet */
    CHECK_EQ_I(r.fake.locks_held, 0);

    /* Gateway sync arrives: synced event, later samples are synced. */
    feed_sync(&r, 0, 0x1234, 5000000);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_TIME_SYNCED), 1);
    from = r.fake.n_tx;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    CHECK(lbr_can_decode_sample(&r.fake.tx[from], 0, &s));
    CHECK(s.synced);

    lbr_health_record_t h;
    from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(r.fake.n_tx - from, LBR_HEALTH_PAGE_COUNT);
    CHECK_EQ_U(h.state, LBR_NODE_RUN);
    CHECK((h.flags & LBR_HF_TIME_SYNCED) != 0);
    CHECK_EQ_U(h.sensor_status, 1);
    CHECK_EQ_U(h.boot_count, 7);
}

static void test_node_sensor_fault_and_recovery(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    lbr_node_core_start(&r.core, true, false, NULL);
    lbr_node_core_slot_start(&r.core, 0);

    r.fs.fail_read = 1;
    size_t from    = r.fake.n_tx;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_SENSOR_INIT_FAIL), 1);
    lbr_sample_t s;
    CHECK(lbr_can_decode_sample(&r.fake.tx[r.fake.n_tx - 1], 0, &s));
    CHECK(!s.valid);
    CHECK_EQ_U(s.v.u32, LBR_UNAVAIL_SENSOR_FAULT); /* never a fake zero */

    lbr_health_record_t h;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.state, LBR_NODE_DEGRADED);
    CHECK_EQ_U(h.sensor_status, 0);

    /* No retry before the back-off, then recovery. */
    int inits = r.fs.inits;
    r.fake.mono += LBR_NODE_SENSOR_RETRY_US - 1;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    CHECK_EQ_I(r.fs.inits, inits);
    r.fake.mono += 1;
    from = r.fake.n_tx;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    CHECK_EQ_I(r.fs.inits, inits + 1);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_SENSOR_RECOVERED), 1);
}

static void test_node_rejects_bad_config(void)
{
    static node_rig_t r;
    CHECK(!rig_init(&r, 0)); /* rate 0 */
    CHECK(r.core.config_error[0] != '\0');
    lbr_node_core_start(&r.core, true, true, NULL);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_FAULT, LBR_EVT_CONFIG_INVALID), 1);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_FAULT, LBR_EVT_CAN_PINS_UNSET), 1);
    lbr_health_record_t h;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.state, LBR_NODE_CONFIG_INVALID);
}

static void test_node_counts_overruns_and_bus_off(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    lbr_node_core_start(&r.core, true, false, NULL);
    lbr_node_core_slot_start(&r.core, 0);
    lbr_node_core_slot_tick(&r.core, 0, 3);

    r.fake.can.bus_off_count = 1;
    size_t              from = r.fake.n_tx;
    lbr_health_record_t h;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.overflow_count, 3);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_CAN_BUS_OFF), 1);
    from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h); /* same count: no second event */
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_CAN_BUS_OFF), 0);

    /* CAN down: nothing is sent, the node reports itself faulted. */
    r.fake.can.running = false;
    r.core.can_up      = false;
    from               = r.fake.n_tx;
    lbr_node_core_slot_tick(&r.core, 0, 0);
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(r.fake.n_tx, from);
    CHECK_EQ_U(h.state, LBR_NODE_FAULT);
}

/* ------------------------------------------------------------------------ */
/* Gateway core                                                              */

typedef struct {
    int     sends;
    size_t  last_len;
    uint8_t last[LBR_PKT_MAX_SIZE];
} fake_link_t;

static int fl_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    fake_link_t *l = self->ctx;
    l->sends++;
    l->last_len = len < sizeof l->last ? len : sizeof l->last;
    memcpy(l->last, pkt, l->last_len);
    return 0;
}

typedef struct {
    uint8_t *buf;
    size_t   len, cap;
} mem_t;

static int mem_write(void *ctx, const uint8_t *b, size_t n)
{
    mem_t *m = ctx;
    if (m->len + n > m->cap) {
        return -1;
    }
    memcpy(m->buf + m->len, b, n);
    m->len += n;
    return 0;
}

static int mem_ok(void *ctx)
{
    (void)ctx;
    return 0;
}

typedef struct {
    fake_t          fake;
    lbr_gw_core_t   core;
    lbr_logger_t    log;
    mem_t           disk;
    fake_link_t     usb_l, lora_l;
    lbr_transport_t usb, lora;
} gw_rig_t;

static uint8_t s_disk[512 * 1024];
static uint8_t s_q[64 * 1024];
static uint8_t s_chunk[LBR_LOG_MAX_RECORD + 512];

static void gw_rig_init(gw_rig_t *r)
{
    memset(r, 0, sizeof *r);
    fake_init(&r->fake);
    lbr_time_servo_set_master(&r->fake.servo);
    r->fake.sd         = true;
    r->fake.lora_ready = true;

    lbr_log_config_t lc = {
        .queue_mem        = s_q,
        .queue_cap        = sizeof s_q,
        .chunk            = s_chunk,
        .chunk_cap        = sizeof s_chunk,
        .sync_interval_ms = 1000,
    };
    CHECK_EQ_I(lbr_log_init(&r->log, &lc), 0);
    r->disk                   = (mem_t){s_disk, 0, sizeof s_disk};
    lbr_log_storage_t st      = {mem_write, mem_ok, mem_ok, &r->disk};
    lbr_log_session_t session = {.node_id = LBR_NODE_GATEWAY, .config_text = "node=gateway\n"};
    CHECK_EQ_I(lbr_log_open(&r->log, &st, &session), 0);

    r->usb =
        (lbr_transport_t){.name = "usb", .kind = LBR_LINK_USB, .send = fl_send, .ctx = &r->usb_l};
    r->lora = (lbr_transport_t){
        .name = "lora", .kind = LBR_LINK_LORA, .send = fl_send, .ctx = &r->lora_l};

    lbr_gw_hal_t hal = {
        .ctx             = &r->fake,
        .mono_us         = f_mono,
        .now_us          = f_now,
        .local_to_shared = f_l2s,
        .can_send        = f_send,
        .can_stats       = f_stats,
        .sync_arm        = f_sync_arm,
        .sd_present      = f_sd,
        .lora_ready      = f_lora_ready,
        .lock            = f_gw_lock,
        .unlock          = f_gw_unlock,
        .log             = f_log,
    };
    lbr_gw_config_t cfg;
    lbr_gw_config_defaults(&cfg);
    lbr_gw_boot_t boot = {
        .reset_reason = LBR_RESET_POWER_ON,
        .boot_count   = 3,
        .epoch        = 0x2345,
        .config_valid = true,
        .config_text  = "node=gateway\n",
    };
    lbr_gw_core_init(&r->core, &hal, &cfg, &boot);
    lbr_gw_links_t links = {.usb = &r->usb, .lora = &r->lora};
    lbr_gw_core_start(&r->core, &r->log, &links, true, false);
}

/* Count CAN_FRAME records in the log (pumped to the fake disk). */
static int logged_frames(gw_rig_t *r, bool own)
{
    lbr_log_pump(&r->log, r->fake.mono);
    lbr_log_reader_t rd;
    lbr_log_record_t rec;
    int              n = 0;
    lbr_log_reader_init(&rd, r->disk.buf, r->disk.len);
    while (lbr_log_reader_next(&rd, &rec)) {
        if (rec.type == LBR_REC_CAN_FRAME) {
            uint64_t        t;
            uint8_t         flags;
            lbr_can_frame_t f;
            lbr_can_frame_rec_decode(rec.payload, rec.len, &t, &flags, &f);
            n += ((flags & LBR_CAN_REC_FLAG_OWN_TX) != 0) == own;
        }
    }
    return n;
}

static void test_gw_logs_everything_first(void)
{
    static gw_rig_t r;
    gw_rig_init(&r);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_EVENT, LBR_EVT_BOOT), 1);

    /* A node sample: logged, the node becomes known, streamed to USB. */
    lbr_sample_t s = {
        .signal    = LBR_SIG_SUSP_TRAVEL_FL,
        .source    = LBR_NODE_SUSPENSION,
        .type      = LBR_VAL_F32,
        .valid     = true,
        .synced    = true,
        .seq       = 1,
        .t_mono_us = r.fake.mono,
        .v.f32     = 3.0f,
    };
    lbr_can_frame_t f;
    CHECK(lbr_can_encode_sample(&s, &f));
    lbr_gw_core_on_rx(&r.core, &f, r.fake.mono);
    CHECK(lbr_node_table_find(&r.core.nodes, LBR_NODE_SUSPENSION) != NULL);
    CHECK(strstr(r.fake.last_log, "suspension") != NULL);
    CHECK_EQ_I(logged_frames(&r, false), 1);
    CHECK(logged_frames(&r, true) >= 1); /* our own BOOT event */

    CHECK_EQ_I(r.usb_l.sends, 0);
    lbr_gw_core_stream_flush(&r.core);
    CHECK_EQ_I(r.usb_l.sends, 1);
    lbr_pkt_header_t hdr;
    CHECK(lbr_pkt_parse(r.usb_l.last, r.usb_l.last_len, &hdr, NULL, NULL));
    CHECK(hdr.count >= 2);
    CHECK_EQ_I(r.fake.locks_held, 0);
}

static void test_gw_time_sync(void)
{
    static gw_rig_t r;
    gw_rig_init(&r);
    size_t  from = r.fake.n_tx;
    uint8_t seq  = lbr_gw_core_sync_send(&r.core);
    CHECK_EQ_I(r.fake.sync_arms, 1);
    uint8_t  rs;
    uint16_t epoch;
    CHECK(lbr_can_decode_time_sync(&r.fake.tx[from], &rs, &epoch));
    CHECK_EQ_U(rs, seq);
    CHECK_EQ_U(epoch, 0x2345);

    lbr_gw_core_sync_done(&r.core, true, 7777777);
    uint64_t tx;
    CHECK(lbr_can_decode_time_follow_up(&r.fake.tx[from + 1], &rs, &tx));
    CHECK_EQ_U(rs, seq);
    CHECK_EQ_U(tx, 7777777);

    /* A sync that never went out: counted, and the next one moves on. */
    uint8_t seq2 = lbr_gw_core_sync_send(&r.core);
    CHECK_EQ_U((uint8_t)(seq2 - seq), 1);
    lbr_gw_core_sync_done(&r.core, false, 0);
    lbr_health_record_t h;
    lbr_gw_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.transport_error_count, 1);
    CHECK((h.flags & LBR_HF_TIME_MASTER) != 0);
}

static void test_gw_node_lost_and_lora(void)
{
    static gw_rig_t r;
    gw_rig_init(&r);
    lbr_health_record_t nh = {.node_id = LBR_NODE_TEMPERATURE, .state = LBR_NODE_RUN};
    lbr_can_frame_t     f;
    lbr_can_encode_health_page(LBR_NODE_TEMPERATURE, &nh, 0, &f);
    lbr_gw_core_on_rx(&r.core, &f, r.fake.mono);

    lbr_gw_core_lora_tick(&r.core);
    CHECK_EQ_I(r.lora_l.sends, 1);
    r.fake.lora_ready = false; /* duty budget spent: nothing sent, nothing blocks */
    lbr_gw_core_lora_tick(&r.core);
    CHECK_EQ_I(r.lora_l.sends, 1);

    lbr_health_record_t h;
    lbr_gw_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.state, LBR_NODE_RUN);
    size_t from = r.fake.n_tx;
    r.fake.mono += (uint64_t)r.core.cfg.node_timeout_ms * 1000u + 1000u;
    lbr_gw_core_heartbeat(&r.core, &h);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_NODE_LOST), 1);

    char buf[1024];
    CHECK(lbr_gw_core_status(&r.core, "00000001.LBR", buf, sizeof buf) > 0);
    CHECK(strstr(buf, "00000001.LBR") != NULL);

    /* No card: still running, degraded, never silent. */
    lbr_log_close(&r.log, r.fake.mono);
    r.fake.sd = false;
    lbr_gw_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.state, LBR_NODE_DEGRADED);
    CHECK((h.flags & LBR_HF_SD_PRESENT) == 0);
}

int main(void)
{
    RUN(test_node_boot_and_acquire);
    RUN(test_node_sensor_fault_and_recovery);
    RUN(test_node_rejects_bad_config);
    RUN(test_node_counts_overruns_and_bus_off);
    RUN(test_gw_logs_everything_first);
    RUN(test_gw_time_sync);
    RUN(test_gw_node_lost_and_lora);
    TEST_MAIN_END();
}
