/*
 * The platform-independent node and gateway cores, driven through fake HALs:
 * the same logic the ESP32 glue and the simulator run.
 */
#include <stdlib.h>

#include "lbr/dash_link.h"
#include "lbr/gw_core.h"
#include "lbr/wlink.h"
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
        if (lbr_can_decode_event(&f->tx[i], 0, &s) && s.signal == sig &&
            (s.v.u32 & 0xFFFFu) == code) {
            n++;
        }
    }
    return n;
}

/* Index of the first frame with `id` since `from`, or -1. */
static int find_frame(const fake_t *f, size_t from, uint32_t id)
{
    for (size_t i = from; i < f->n_tx; i++) {
        if (f->tx[i].id == id) {
            return (int)i;
        }
    }
    return -1;
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

static const lbr_signal_id_t k_sigs[] = {LBR_SIG_ENGINE_RPM, LBR_SIG_CVT_RPM};

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
        .name      = "speeds",
        .signals   = k_sigs,
        .n_signals = 2,
        .init      = fs_init,
        .read      = fs_read,
        .ctx       = &r->fs,
    };
    r->acq = (lbr_acq_t){.sensor = &r->sensor, .rate_hz = rate_hz};
    r->cfg = (lbr_node_config_t){
        .node_id          = LBR_NODE_POWERTRAIN,
        .acq              = &r->acq,
        .n_acq            = 1,
        .health_period_ms = 1000,
    };
    lbr_node_hal_t  hal  = node_hal(&r->fake);
    lbr_node_boot_t boot = {
        .reset_reason = LBR_RESET_POWER_ON,
        .boot_count   = 7,
        .can_bitrate  = 500000,
    };
    return lbr_node_core_init(&r->core, &r->cfg, &hal, &boot);
}

/* Feed the node a Central TIME_SYNC + FOLLOW_UP pair. */
static void feed_sync(node_rig_t *r, uint8_t seq, uint16_t epoch, uint64_t gw_us)
{
    lbr_can_frame_t f;
    lbr_can_encode_time_sync(seq, epoch, gw_us, &f);
    lbr_node_core_on_frame(&r->core, &f, r->fake.mono);
    lbr_can_encode_time_follow_up(seq, gw_us, &f);
    lbr_node_core_on_frame(&r->core, &f, r->fake.mono + 50);
}

/* Advance the clock to slot 0's next grid point and tick it. */
static void tick(node_rig_t *r)
{
    uint64_t due = lbr_node_core_slot_next_due(&r->core, 0);
    uint64_t now = lbr_time_servo_now(&r->fake.servo, r->fake.mono);
    r->fake.mono += due - now;
    lbr_node_core_slot_tick(&r->core, 0);
}

/* ------------------------------------------------------------------------ */
/* Node core                                                                 */

static void test_node_boot_and_acquire(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    CHECK(strstr(r.fake.last_log, "powertrain") != NULL);
    lbr_node_core_start(&r.core, true, false, NULL);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_EVENT, LBR_EVT_BOOT), 1);

    lbr_node_core_slot_start(&r.core, 0);
    r.fs.value  = 3000.0f;
    size_t from = r.fake.n_tx;
    tick(&r);
    CHECK_EQ_U(r.fake.n_tx - from, 1); /* both speeds in one POWERTRAIN_SPEEDS frame */
    CHECK_EQ_U(r.fake.tx[from].id, 0x200);
    lbr_sample_t s[4];
    uint64_t     t = lbr_time_servo_now(&r.fake.servo, r.fake.mono);
    CHECK_EQ_U(lbr_can_decode_data(&r.fake.tx[from], t + 700, false, s, 4), 2u);
    CHECK_EQ_U(s[0].signal, LBR_SIG_ENGINE_RPM);
    CHECK_EQ_U(s[0].source, LBR_NODE_POWERTRAIN);
    CHECK(s[0].valid && s[0].v.f32 == 3000.0f);
    CHECK(s[1].valid && s[1].v.f32 == 3001.0f);
    CHECK_EQ_U(s[0].t_mono_us, t); /* the grid point it was sampled at */
    CHECK_EQ_I(r.fake.locks_held, 0);

    /* The Central's sync arrives: synced event, a new timeline. */
    feed_sync(&r, 0, 0x1234, 5000000);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_TIME_SYNCED), 1);
    from = r.fake.n_tx;
    tick(&r);
    CHECK_EQ_I(find_frame(&r.fake, from, 0x200), (int)from);
    CHECK_EQ_U(r.core.overruns, 0u); /* the clock jump is not counted as overruns */

    lbr_health_record_t h;
    from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(r.fake.n_tx - from, 1u + LBR_HEALTH_PAGE_COUNT);
    uint8_t state, flags;
    CHECK_EQ_U(lbr_can_decode_heartbeat(&r.fake.tx[from], &state, &flags), LBR_NODE_POWERTRAIN);
    CHECK_EQ_U(state, LBR_NODE_RUN);
    CHECK_EQ_U(flags, 0u);
    CHECK_EQ_U(r.fake.tx[from + 1].id, 0x780u + 16u * LBR_NODE_POWERTRAIN);
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
    tick(&r);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_SENSOR_INIT_FAIL), 1);
    int          at = find_frame(&r.fake, from, 0x200);
    lbr_sample_t s[2];
    CHECK(at >= 0);
    CHECK_EQ_U(lbr_can_decode_data(&r.fake.tx[at], r.fake.mono, false, s, 2), 2u);
    CHECK(!s[0].valid);
    CHECK_EQ_U(s[0].v.u32, LBR_UNAVAIL_SENSOR_FAULT); /* never a fake zero */

    lbr_health_record_t h;
    from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.state, LBR_NODE_DEGRADED);
    CHECK_EQ_U(h.sensor_status, 0);
    uint8_t state, flags;
    lbr_can_decode_heartbeat(&r.fake.tx[from], &state, &flags);
    CHECK((flags & LBR_HB_ERR_SENSOR) != 0);
    CHECK((flags & LBR_HB_ERR_NOT_SYNCED) != 0);

    /* No retry before the back-off, then recovery. */
    int inits = r.fs.inits;
    r.fake.mono += LBR_ACQ_RETRY_US - 20000;
    tick(&r);
    CHECK_EQ_I(r.fs.inits, inits);
    r.fake.mono += 20000;
    from = r.fake.n_tx;
    tick(&r);
    CHECK_EQ_I(r.fs.inits, inits + 1);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_SENSOR_RECOVERED), 1);
}

static void test_node_rejects_bad_config(void)
{
    static node_rig_t r;
    CHECK(!rig_init(&r, 50)); /* POWERTRAIN_SPEEDS is a 100 Hz frame */
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
    tick(&r);
    r.fake.mono += 30000; /* three grid points go by unsampled */
    tick(&r);

    r.fake.can.bus_off_count = 1;
    size_t              from = r.fake.n_tx;
    lbr_health_record_t h;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.overflow_count, 3);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_CAN_BUS_OFF), 1);
    uint8_t state = 0, flags = 0;
    int     hb = find_frame(&r.fake, from, 0x700u + LBR_NODE_POWERTRAIN);
    CHECK(hb >= 0 && lbr_can_decode_heartbeat(&r.fake.tx[hb], &state, &flags) != 0);
    CHECK((flags & (LBR_HB_ERR_CAN | LBR_HB_ERR_LOSS)) == (LBR_HB_ERR_CAN | LBR_HB_ERR_LOSS));
    from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h); /* same count: no second event, no new loss */
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_CAN_BUS_OFF), 0);
    lbr_can_decode_heartbeat(&r.fake.tx[from], &state, &flags);
    CHECK((flags & (LBR_HB_ERR_CAN | LBR_HB_ERR_LOSS)) == 0);

    /* CAN down: nothing is sent, the node reports itself faulted. */
    r.fake.can.running = false;
    r.core.can_up      = false;
    from               = r.fake.n_tx;
    tick(&r);
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(r.fake.n_tx, from);
    CHECK_EQ_U(h.state, LBR_NODE_FAULT);
}

static int  s_app_frames;
static void app_frame(const lbr_can_frame_t *f, uint64_t rx_shared_us, void *ctx)
{
    (void)f;
    (void)rx_shared_us;
    (void)ctx;
    s_app_frames++;
}

static void test_node_app_hooks(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    r.cfg.on_frame = app_frame;
    lbr_node_core_start(&r.core, true, false, NULL);

    /* Commands are consumed by the runtime, other frames go to the app. */
    uint8_t         args[7] = {0};
    lbr_can_frame_t f;
    lbr_can_encode_command(LBR_CMD_SESSION_START, args, &f);
    lbr_node_core_on_frame(&r.core, &f, r.fake.mono);
    CHECK_EQ_U(r.core.commands, 1u);
    CHECK_EQ_I(s_app_frames, 0);
    f.id = 0x300;
    lbr_node_core_on_frame(&r.core, &f, r.fake.mono);
    CHECK_EQ_I(s_app_frames, 1);

    /* Forwarded frames count as published; app flags and losses reach the
     * heartbeat. */
    unsigned pub = r.core.published;
    CHECK(lbr_node_core_send(&r.core, &f));
    CHECK_EQ_U(r.core.published, pub + 1u);
    lbr_node_core_set_app_flags(&r.core, LBR_HB_ERR_LINK);
    lbr_node_core_count_loss(&r.core, 5);
    lbr_health_record_t h;
    size_t              from = r.fake.n_tx;
    lbr_node_core_heartbeat(&r.core, &h);
    uint8_t state, flags;
    lbr_can_decode_heartbeat(&r.fake.tx[from], &state, &flags);
    CHECK((flags & LBR_HB_ERR_LINK) != 0);
    CHECK((flags & LBR_HB_ERR_LOSS) != 0);
    CHECK_EQ_U(h.transport_error_count, 5u);
    CHECK_EQ_U(h.state, LBR_NODE_DEGRADED);
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
    fake_link_t     usb_l, lora_l, dash_l;
    lbr_transport_t usb, lora, dash;
    lbr_sensor_t    dash_status;
    lbr_acq_t       acq;
} gw_rig_t;

static const lbr_signal_id_t k_dash_status[] = {LBR_SIG_DASH_BATTERY, LBR_SIG_DASH_LINK};

static uint8_t s_disk[512 * 1024];
static uint8_t s_q[64 * 1024];
static uint8_t s_chunk[LBR_LOG_MAX_RECORD + 512];

static void gw_rig_start(gw_rig_t *r)
{
    lbr_gw_links_t links = {.usb = &r->usb, .lora = &r->lora, .dash = &r->dash};
    lbr_gw_core_start(&r->core, &r->log, &links, true, false);
    lbr_gw_core_slot_start(&r->core, 0);
}

static void gw_rig_setup(gw_rig_t *r)
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
    lbr_log_session_t session = {.node_id = LBR_NODE_CENTRAL, .config_text = "node=central\n"};
    CHECK_EQ_I(lbr_log_open(&r->log, &st, &session), 0);

    r->usb =
        (lbr_transport_t){.name = "usb", .kind = LBR_LINK_USB, .send = fl_send, .ctx = &r->usb_l};
    r->lora = (lbr_transport_t){
        .name = "lora", .kind = LBR_LINK_LORA, .send = fl_send, .ctx = &r->lora_l};
    r->dash = (lbr_transport_t){
        .name = "dash", .kind = LBR_LINK_DASH, .send = fl_send, .ctx = &r->dash_l};

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
    /* The Central's dashboard-status slot reads the core's own link state. */
    r->acq             = (lbr_acq_t){.sensor = &r->dash_status, .rate_hz = 1};
    lbr_gw_boot_t boot = {
        .reset_reason = LBR_RESET_POWER_ON,
        .boot_count   = 3,
        .epoch        = 0x2345,
        .config_valid = true,
        .config_text  = "node=central\n",
        .acq          = &r->acq,
        .n_acq        = 1,
    };
    lbr_gw_core_init(&r->core, &hal, &cfg, &boot);
    lbr_sensor_dash_link(&r->dash_status, "dash_status", k_dash_status, 2, &r->core.dash,
                         lbr_gw_core_lock_state, lbr_gw_core_unlock_state, &r->core);
}

static void gw_rig_init(gw_rig_t *r)
{
    gw_rig_setup(r);
    gw_rig_start(r);
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
    CHECK(find_frame(&r.fake, 0, LBR_CAN_ID_COMMAND) >= 0); /* SESSION_START: log is open */

    /* A node's data frame: logged, the node becomes known, streamed to USB. */
    lbr_sample_t v[2] = {
        {.signal = LBR_SIG_ENGINE_RPM, .type = LBR_VAL_F32, .valid = true, .v.f32 = 3100.0f},
        {.signal = LBR_SIG_CVT_RPM, .type = LBR_VAL_F32, .valid = true, .v.f32 = 900.0f},
    };
    lbr_can_frame_t f;
    lbr_can_encode_data(lbr_frame_def(0x200), v, r.fake.mono / 10000u, &f);
    lbr_gw_core_on_rx(&r.core, &f, r.fake.mono);
    CHECK(lbr_node_table_find(&r.core.nodes, LBR_NODE_POWERTRAIN) != NULL);
    CHECK(strstr(r.fake.last_log, "powertrain") != NULL);
    CHECK_EQ_I(logged_frames(&r, false), 1);
    CHECK(logged_frames(&r, true) >= 2); /* our own BOOT event and SESSION_START */

    CHECK_EQ_I(r.usb_l.sends, 0);
    lbr_gw_core_stream_flush(&r.core);
    CHECK_EQ_I(r.usb_l.sends, 1);
    lbr_pkt_header_t hdr;
    CHECK(lbr_pkt_parse(r.usb_l.last, r.usb_l.last_len, &hdr, NULL, NULL));
    CHECK(hdr.count >= 3);
    CHECK_EQ_I(r.fake.locks_held, 0);

    /* The decoded value reaches the dashboard's view. */
    lbr_sample_t got;
    CHECK_EQ_I(lbr_value_table_get(&r.core.dash_values, LBR_SIG_ENGINE_RPM, r.fake.mono, &got),
               LBR_VALUE_OK);
    CHECK(got.v.f32 == 3100.0f);
}

typedef struct {
    int  samples, lites;
    bool rpm;
} dash_seen_t;

static void on_dash_rec(uint8_t type, const uint8_t *p, size_t len, void *ctx)
{
    dash_seen_t *d = ctx;
    if (type == LBR_REC_SAMPLE) {
        lbr_sample_t s;
        if (lbr_sample_decode(p, len, &s) > 0) {
            d->samples++;
            d->rpm = d->rpm || (s.signal == LBR_SIG_ENGINE_RPM && s.valid);
        }
    } else if (type == LBR_REC_HEALTH_LITE) {
        d->lites++;
    }
}

static void test_gw_dashboard_link(void)
{
    static gw_rig_t r;
    gw_rig_init(&r);

    /* Before the dashboard is heard: DASH_STATUS says link down, battery NO_DATA. */
    r.fake.mono = 2000000;
    size_t from = r.fake.n_tx;
    lbr_gw_core_slot_tick(&r.core, 0);
    int          at = find_frame(&r.fake, from, 0x60A);
    lbr_sample_t s[2];
    CHECK(at >= 0);
    CHECK_EQ_U(lbr_can_decode_data(&r.fake.tx[at], r.fake.mono, true, s, 2), 2u);
    CHECK(!s[0].valid && s[0].v.u32 == LBR_UNAVAIL_NO_DATA);
    CHECK(s[1].valid && !s[1].v.b);

    /* The dashboard reports 76 % battery. */
    uint8_t           pkt[128];
    lbr_pkt_builder_t b;
    lbr_sample_t      batt = {.signal = LBR_SIG_DASH_BATTERY,
                              .source = LBR_NODE_DASHBOARD,
                              .type   = LBR_VAL_F32,
                              .valid  = true,
                              .v.f32  = 76.0f};
    uint8_t           rec[LBR_SAMPLE_WIRE_SIZE];
    lbr_sample_encode(&batt, rec, sizeof rec);
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_DASH, LBR_NODE_DASHBOARD, 0, 0);
    lbr_pkt_add(&b, LBR_REC_SAMPLE, rec, sizeof rec);
    size_t n = lbr_pkt_finish(&b);
    lbr_gw_core_on_dash_packet(&r.core, pkt, n, r.fake.mono);
    from = r.fake.n_tx;
    lbr_gw_core_dash_tick(&r.core);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_NODE_FOUND), 1);

    r.fake.mono = 3000000;
    lbr_gw_core_on_dash_packet(&r.core, pkt, n, r.fake.mono); /* it keeps reporting */
    lbr_gw_core_dash_tick(&r.core);
    from = r.fake.n_tx;
    lbr_gw_core_slot_tick(&r.core, 0);
    at = find_frame(&r.fake, from, 0x60A);
    CHECK(at >= 0);
    lbr_can_decode_data(&r.fake.tx[at], r.fake.mono, true, s, 2);
    CHECK(s[0].valid && s[0].v.f32 == 76.0f);
    CHECK(s[1].valid && s[1].v.b);

    /* Display datagrams carry the latest values and node health. */
    lbr_sample_t v[2] = {
        {.signal = LBR_SIG_ENGINE_RPM, .type = LBR_VAL_F32, .valid = true, .v.f32 = 2500.0f},
        {.signal = LBR_SIG_CVT_RPM, .type = LBR_VAL_F32, .valid = true, .v.f32 = 800.0f},
    };
    lbr_can_frame_t f;
    lbr_can_encode_data(lbr_frame_def(0x200), v, r.fake.mono / 10000u, &f);
    lbr_gw_core_on_rx(&r.core, &f, r.fake.mono);
    int sends = r.dash_l.sends;
    lbr_gw_core_dash_tick(&r.core);
    CHECK_EQ_I(r.dash_l.sends, sends + 1);
    lbr_pkt_header_t hdr;
    dash_seen_t      seen = {0};
    CHECK(lbr_pkt_parse(r.dash_l.last, r.dash_l.last_len, &hdr, on_dash_rec, &seen));
    CHECK_EQ_U(hdr.link, LBR_LINK_DASH);
    CHECK_EQ_U(hdr.source, LBR_NODE_CENTRAL);
    CHECK(seen.rpm);
    CHECK(seen.lites >= 1);

    /* Silence: the link goes down, counted as a lost node. */
    r.fake.mono += LBR_WLINK_DASH_TIMEOUT_US + 1000;
    from = r.fake.n_tx;
    lbr_gw_core_dash_tick(&r.core);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_FAULT, LBR_EVT_NODE_LOST), 1);
    lbr_health_record_t h;
    from = r.fake.n_tx;
    lbr_gw_core_heartbeat(&r.core, &h);
    uint8_t state, flags;
    CHECK_EQ_U(lbr_can_decode_heartbeat(&r.fake.tx[from], &state, &flags), LBR_NODE_CENTRAL);
    CHECK((flags & LBR_HB_ERR_LINK) != 0);
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
    r.core.links.dash      = NULL; /* no dashboard in this test */
    lbr_can_frame_t f;
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
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_EVENT, LBR_EVT_NODE_FOUND), 0); /* first seen */
    /* Back on the bus: NODE_FOUND, so the log shows the outage ended. */
    lbr_gw_core_on_rx(&r.core, &f, r.fake.mono);
    CHECK_EQ_I(count_events(&r.fake, from, LBR_SIG_EVENT, LBR_EVT_NODE_FOUND), 1);

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

/* Events raised before start (driver init, config checks) are held and go
 * out right after BOOT, with the time they happened, never lost. */
static void test_early_events(void)
{
    static node_rig_t r;
    CHECK(rig_init(&r, 100));
    r.fake.mono = 4000;
    for (unsigned i = 0; i < LBR_EARLY_EVENTS + 3u; i++) {
        lbr_node_core_event(&r.core, LBR_EVT_SENSOR_INIT_FAIL, (uint16_t)i, true);
    }
    CHECK_EQ_U(r.fake.n_tx, 0u);
    r.fake.mono = 9000;
    lbr_node_core_start(&r.core, true, false, NULL);
    lbr_sample_t e;
    CHECK(r.fake.n_tx >= 1u + LBR_EARLY_EVENTS);
    CHECK(lbr_can_decode_event(&r.fake.tx[0], 9000, &e));
    CHECK_EQ_U(e.v.u32 & 0xFFFFu, LBR_EVT_BOOT);
    CHECK(lbr_can_decode_event(&r.fake.tx[1], 9000, &e));
    CHECK_EQ_U(e.signal, LBR_SIG_FAULT);
    CHECK_EQ_U(e.v.u32, lbr_event_value(LBR_EVT_SENSOR_INIT_FAIL, 0));
    CHECK_EQ_U(e.t_mono_us, 4000u);
    CHECK_EQ_I(count_events(&r.fake, 0, LBR_SIG_FAULT, LBR_EVT_SENSOR_INIT_FAIL),
               (int)LBR_EARLY_EVENTS);
    lbr_health_record_t h;
    lbr_node_core_heartbeat(&r.core, &h);
    CHECK_EQ_U(h.overflow_count, 3u); /* the ones that did not fit are counted */

    static gw_rig_t g;
    gw_rig_setup(&g);
    lbr_gw_core_event(&g.core, LBR_EVT_CONFIG_INVALID, 0, true);
    CHECK_EQ_U(g.fake.n_tx, 0u);
    gw_rig_start(&g);
    CHECK_EQ_I(count_events(&g.fake, 0, LBR_SIG_EVENT, LBR_EVT_BOOT), 1);
    CHECK_EQ_I(count_events(&g.fake, 1, LBR_SIG_FAULT, LBR_EVT_CONFIG_INVALID), 1);
}

int main(void)
{
    RUN(test_node_boot_and_acquire);
    RUN(test_node_sensor_fault_and_recovery);
    RUN(test_node_rejects_bad_config);
    RUN(test_node_counts_overruns_and_bus_off);
    RUN(test_node_app_hooks);
    RUN(test_gw_logs_everything_first);
    RUN(test_gw_time_sync);
    RUN(test_gw_dashboard_link);
    RUN(test_gw_node_lost_and_lora);
    RUN(test_early_events);
    TEST_MAIN_END();
}
