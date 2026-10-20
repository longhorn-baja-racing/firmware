#include "lbr/acq.h"
#include "lbr/node.h"
#include "lbr/node_table.h"
#include "lbr/nodes.h"
#include "lbr/value_table.h"
#include "lbr_test.h"

static const lbr_signal_id_t k_speeds[]  = {LBR_SIG_ENGINE_RPM, LBR_SIG_CVT_RPM};
static const lbr_signal_id_t k_fuel[]    = {LBR_SIG_FUEL_LEVEL};
static const lbr_signal_id_t k_half[]    = {LBR_SIG_ENGINE_RPM}; /* 0x200 also needs CVT_RPM */
static const lbr_signal_id_t k_foreign[] = {LBR_SIG_TEMP_CVT};   /* sent by temperature */

static lbr_sensor_t s_speeds;
static lbr_sensor_t s_other;

static lbr_node_config_t base_config(lbr_acq_t *acq)
{
    lbr_sensor_unconfigured(&s_speeds, "speeds", k_speeds, 2);
    acq[0]                = (lbr_acq_t){&s_speeds, 100};
    lbr_node_config_t cfg = {
        .node_id          = LBR_NODE_POWERTRAIN,
        .acq              = acq,
        .n_acq            = 1,
        .health_period_ms = 1000,
    };
    return cfg;
}

static void test_valid_config(void)
{
    lbr_acq_t         acq[2];
    lbr_node_config_t cfg = base_config(acq);
    char              err[128];
    CHECK_EQ_I(lbr_node_config_validate(&cfg, err, sizeof err), 0);
    lbr_sensor_unconfigured(&s_other, "fuel", k_fuel, 1);
    acq[1]    = (lbr_acq_t){&s_other, 1};
    cfg.n_acq = 2;
    CHECK_EQ_I(lbr_node_config_validate(&cfg, err, sizeof err), 0);
    /* The load is the frame table's, whatever the slots: 0x200 at 100 Hz,
     * 0x201 at 1 Hz, heartbeat + 12 health pages once a second. */
    uint32_t expect = 100u * lbr_can_frame_bits_worst(8) + lbr_can_frame_bits_worst(2) +
                      lbr_can_frame_bits_worst(2) + 12u * lbr_can_frame_bits_worst(8);
    CHECK_EQ_U(lbr_node_can_load_bps(&cfg), expect);
}

static void test_rejects(void)
{
    lbr_acq_t         acq[2];
    char              err[160] = "";
    lbr_node_config_t cfg      = base_config(acq);

    cfg.node_id = LBR_NODE_CENTRAL;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    cfg.node_id = LBR_NODE_CORNER_FL;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    cfg = base_config(acq);

    /* The slot rate must be its frame's rate, and divide 1 MHz. */
    acq[0].rate_hz = 0;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    acq[0].rate_hz = 200;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    CHECK(strstr(err, "POWERTRAIN_SPEEDS") != NULL);
    acq[0].rate_hz = 3;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    cfg = base_config(acq);

    /* A frame's fields must all come from one slot. */
    lbr_sensor_unconfigured(&s_speeds, "half", k_half, 1);
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    CHECK(strstr(err, "CVT_RPM") != NULL);
    cfg = base_config(acq);

    /* A signal in another node's frame. */
    lbr_sensor_unconfigured(&s_other, "foreign", k_foreign, 1);
    acq[1]    = (lbr_acq_t){&s_other, 10};
    cfg.n_acq = 2;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    CHECK(strstr(err, "temperature") != NULL);
    cfg = base_config(acq);

    /* Same signal from two drivers (each slot complete on its own, so only
     * the duplicate can be the reason). */
    lbr_sensor_unconfigured(&s_other, "dup", k_speeds, 2);
    acq[1]    = (lbr_acq_t){&s_other, 100};
    cfg.n_acq = 2;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
    CHECK(strstr(err, "twice") != NULL);
    cfg = base_config(acq);

    cfg.listen_all = true;
    CHECK(lbr_node_config_validate(&cfg, err, sizeof err) < 0);
}

static void test_config_text_and_crc(void)
{
    lbr_acq_t         acq[2];
    lbr_node_config_t cfg = base_config(acq);
    char              text[512];
    size_t            n = lbr_node_config_text(&cfg, text, sizeof text);
    CHECK(n > 0 && n < sizeof text);
    CHECK(strstr(text, "node=powertrain\n") != NULL);
    CHECK(strstr(text, "sensor.0=speeds rate_hz=100 sim=0 signals=ENGINE_RPM,CVT_RPM\n") != NULL);
    uint32_t crc   = lbr_node_config_crc(&cfg);
    acq[0].rate_hz = 50;
    CHECK(lbr_node_config_crc(&cfg) != crc);
    /* Truncation is safe. */
    char small[8];
    CHECK_EQ_U(lbr_node_config_text(&cfg, small, sizeof small), 7u);
    CHECK_EQ_U(small[7], 0);
}

static void test_slot_frames(void)
{
    const lbr_frame_def_t       *frames[4];
    uint8_t                      chan[4][LBR_FRAME_MAX_FIELDS];
    static const lbr_signal_id_t rev[] = {LBR_SIG_CVT_RPM, LBR_SIG_ENGINE_RPM};
    CHECK_EQ_U(lbr_slot_frames(LBR_NODE_POWERTRAIN, rev, 2, frames, chan, 4), 1u);
    CHECK_EQ_U(frames[0]->can_id, 0x200);
    CHECK_EQ_U(chan[0][0], 1u); /* ENGINE_RPM is channel 1 of this slot */
    CHECK_EQ_U(chan[0][1], 0u);
    static const lbr_signal_id_t stress[] = {
        LBR_SIG_STRESS_1, LBR_SIG_STRESS_2, LBR_SIG_STRESS_3, LBR_SIG_STRESS_4,
        LBR_SIG_STRESS_5, LBR_SIG_STRESS_6, LBR_SIG_STRESS_7, LBR_SIG_STRESS_8,
    };
    CHECK_EQ_U(lbr_slot_frames(LBR_NODE_STRESS, stress, 8, frames, chan, 4), 2u);
    CHECK_EQ_U(frames[1]->can_id, 0x401);
    CHECK_EQ_U(chan[1][3], 7u);
    /* Not this node's frames. */
    CHECK_EQ_U(lbr_slot_frames(LBR_NODE_STRESS, rev, 2, frames, chan, 4), 0u);
}

static void test_reading_to_sample(void)
{
    lbr_reading_t r = {.valid = true, .v.f32 = 3.0f};
    lbr_sample_t  s;
    lbr_reading_to_sample(LBR_NODE_STRESS, LBR_SIG_STRESS_1, &r, 1234, 10000, true, &s);
    CHECK(s.valid && s.v.f32 == 3.0f && s.type == LBR_VAL_F32 && s.synced);
    CHECK_EQ_U(s.seq, 1234u);
    CHECK_EQ_U(s.t_mono_us, 12340000u); /* the grid time of sample k */
    lbr_reading_t bad = {.valid = false, .reason = LBR_UNAVAIL_BUS_ERROR};
    lbr_reading_to_sample(LBR_NODE_STRESS, LBR_SIG_STRESS_1, &bad, 1, 10000, false, &s);
    CHECK(!s.valid);
    CHECK_EQ_U(s.v.u32, LBR_UNAVAIL_BUS_ERROR);
}

/* A part that fails on demand. */
static int s_fail_init;
static int s_fail_read;
static int s_reads;
static int s_inits;
static int flaky_init(lbr_sensor_t *self)
{
    (void)self;
    s_inits++;
    return s_fail_init ? -1 : 0;
}
static int flaky_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)self;
    s_reads++;
    out[0] = (lbr_reading_t){.valid = true, .v.f32 = (float)(t_us / 1000u)};
    return s_fail_read ? -1 : 0;
}

static int  s_events[0x20];
static void on_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    (void)ctx;
    (void)arg;
    (void)fault;
    s_events[code & 0x1F]++;
}

static void test_acq_grid(void)
{
    lbr_sensor_t sensor = {
        .name = "flaky", .signals = k_fuel, .n_signals = 1, .init = flaky_init, .read = flaky_read};
    lbr_acq_slot_t a;
    lbr_reading_t  r[1];
    lbr_acq_tick_t tick;
    memset(s_events, 0, sizeof s_events);
    s_fail_init = 0;
    s_fail_read = 0;
    lbr_acq_slot_init(&a, &sensor, 100, 3);
    lbr_acq_slot_start(&a, 0, on_event, NULL);
    CHECK(atomic_load(&a.healthy));

    /* Woken just after, and just before, grid points: k is the nearest. */
    CHECK(lbr_acq_slot_tick(&a, 50000 + 120, 0, 1, r, &tick, on_event, NULL));
    CHECK_EQ_U(tick.k, 5u);
    CHECK_EQ_U(tick.missed, 0u);
    CHECK(r[0].valid && r[0].v.f32 == 50.0f); /* the driver reads at the grid time */
    CHECK(lbr_acq_slot_tick(&a, 60000 - 80, 0, 1, r, &tick, on_event, NULL));
    CHECK_EQ_U(tick.k, 6u);
    /* A second wake-up for the same grid point reads nothing. */
    int reads = s_reads;
    CHECK(!lbr_acq_slot_tick(&a, 60000 + 900, 0, 1, r, &tick, on_event, NULL));
    CHECK_EQ_I(s_reads, reads);
    /* Late by two periods: two overruns. */
    CHECK(lbr_acq_slot_tick(&a, 90000, 0, 1, r, &tick, on_event, NULL));
    CHECK_EQ_U(tick.k, 9u);
    CHECK_EQ_U(tick.missed, 2u);
    /* The shared clock jumped (first sync / step): a new timeline, not overruns. */
    CHECK(lbr_acq_slot_tick(&a, 5000000000ull, 0, 2, r, &tick, on_event, NULL));
    CHECK_EQ_U(tick.missed, 0u);
    CHECK_EQ_U(lbr_acq_next_due(5000000000ull + 1, 10000), 5000010000ull);
    /* A backward step that the monotonic clock holds at the point just
     * sampled: a new timeline, but the same k must not be sampled twice. */
    CHECK(!lbr_acq_slot_tick(&a, 5000000000ull, 0, 3, r, &tick, on_event, NULL));

    /* A read failure: SENSOR_FAULT channels, an event, retry after 2 s. */
    s_fail_read = 1;
    s_fail_init = 1;
    CHECK(lbr_acq_slot_tick(&a, 5000010000ull, 100, 3, r, &tick, on_event, NULL));
    CHECK_EQ_U(tick.k, 500001u); /* the next point, nothing missed */
    CHECK_EQ_U(tick.missed, 0u);
    CHECK(!r[0].valid && r[0].reason == LBR_UNAVAIL_SENSOR_FAULT);
    CHECK_EQ_I(s_events[LBR_EVT_SENSOR_INIT_FAIL & 0x1F], 1);
    s_fail_read = 0;
    int inits   = s_inits;
    CHECK(lbr_acq_slot_tick(&a, 5000020000ull, 100 + LBR_ACQ_RETRY_US - 1, 3, r, &tick, on_event,
                            NULL));
    CHECK(!r[0].valid);
    CHECK_EQ_I(s_inits, inits); /* not retried before the wait is over */
    CHECK(
        lbr_acq_slot_tick(&a, 5000030000ull, 100 + LBR_ACQ_RETRY_US, 3, r, &tick, on_event, NULL));
    CHECK(!r[0].valid); /* init still failing */
    CHECK_EQ_I(s_inits, inits + 1);
    s_fail_init = 0;
    CHECK(lbr_acq_slot_tick(&a, 5000040000ull, 100 + 2 * LBR_ACQ_RETRY_US, 3, r, &tick, on_event,
                            NULL));
    CHECK(!r[0].valid); /* recovered this tick; reads resume at the next */
    CHECK_EQ_I(s_events[LBR_EVT_SENSOR_RECOVERED & 0x1F], 1);
    CHECK(lbr_acq_slot_tick(&a, 5000050000ull, 0, 3, r, &tick, on_event, NULL));
    CHECK(r[0].valid);
}

static void test_node_table(void)
{
    lbr_node_table_t t;
    lbr_node_table_init(&t, 3000000);
    lbr_health_record_t h = {.uptime_ms = 10, .state = LBR_NODE_RUN};
    lbr_can_frame_t     f;
    lbr_can_encode_health_page(LBR_NODE_TEMPERATURE, &h, 0, &f);
    CHECK_EQ_I(lbr_node_table_on_frame(&t, &f, 1000), LBR_NT_FOUND);
    CHECK_EQ_I(lbr_node_table_on_frame(&t, &f, 2000), LBR_NT_NONE);
    const lbr_node_entry_t *e = lbr_node_table_find(&t, LBR_NODE_TEMPERATURE);
    CHECK(e != NULL && e->present && e->frames == 2);
    CHECK_EQ_U(e->health.uptime_ms, 10u);
    CHECK_EQ_U(e->pages_seen, 1u);

    uint8_t lost[4];
    CHECK_EQ_U(lbr_node_table_expire(&t, 2000 + 3000000, lost, 4), 0u);
    CHECK_EQ_U(lbr_node_table_expire(&t, 2001 + 3000000, lost, 4), 1u);
    CHECK_EQ_U(lost[0], LBR_NODE_TEMPERATURE);
    CHECK_EQ_U(lbr_node_table_expire(&t, 9000000, lost, 4), 0u);        /* reported once */
    CHECK_EQ_I(lbr_node_table_on_frame(&t, &f, 9500000), LBR_NT_FOUND); /* back */

    /* The heartbeat summary is kept. */
    lbr_can_encode_heartbeat(LBR_NODE_TEMPERATURE, LBR_NODE_DEGRADED, LBR_HB_ERR_SENSOR, &f);
    lbr_node_table_on_frame(&t, &f, 9600000);
    e = lbr_node_table_find(&t, LBR_NODE_TEMPERATURE);
    CHECK(e->hb_seen && e->hb_state == LBR_NODE_DEGRADED && e->hb_flags == LBR_HB_ERR_SENSOR);

    /* Frames of no LBR node are not counted as a node. */
    f.id = 0x7F0;
    CHECK_EQ_I(lbr_node_table_on_frame(&t, &f, 1), LBR_NT_NONE);
    CHECK(lbr_node_table_find(&t, 0) == NULL);
}

static void test_value_table(void)
{
    lbr_value_table_t t;
    lbr_value_table_init(&t, 1000000);
    CHECK(lbr_value_table_watch(&t, LBR_SIG_TEMP_CVT, 0));
    CHECK(lbr_value_table_watch(&t, LBR_SIG_FUEL_LEVEL, LBR_NODE_POWERTRAIN));
    lbr_sample_t out;
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_TEMP_CVT, 0, &out), LBR_VALUE_NONE);

    lbr_sample_t s = {.signal = LBR_SIG_TEMP_CVT,
                      .source = 5,
                      .type   = LBR_VAL_F32,
                      .valid  = true,
                      .v.f32  = 91.0f};
    lbr_value_table_offer(&t, &s, 100);
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_TEMP_CVT, 500000, &out), LBR_VALUE_OK);
    CHECK(out.v.f32 == 91.0f);
    /* Stale data is never presented as current. */
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_TEMP_CVT, 2000000, &out), LBR_VALUE_STALE);
    lbr_sample_set_unavailable(&s, LBR_UNAVAIL_NOT_FITTED);
    lbr_value_table_offer(&t, &s, 2000000);
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_TEMP_CVT, 2000001, &out), LBR_VALUE_INVALID);

    /* Source filter. */
    lbr_sample_t f = {
        .signal = LBR_SIG_FUEL_LEVEL, .source = 9, .type = LBR_VAL_F32, .valid = true};
    lbr_value_table_offer(&t, &f, 10);
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_FUEL_LEVEL, 10, &out), LBR_VALUE_NONE);
    f.source = LBR_NODE_POWERTRAIN;
    lbr_value_table_offer(&t, &f, 10);
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_FUEL_LEVEL, 10, &out), LBR_VALUE_OK);

    for (unsigned i = 2; i < LBR_VALUE_TABLE_MAX; i++) {
        CHECK(lbr_value_table_watch(&t, LBR_SIG_IMU_CH_ACCEL_X, 0));
    }
    CHECK(!lbr_value_table_watch(&t, LBR_SIG_IMU_CH_ACCEL_Y, 0));
}

int main(void)
{
    RUN(test_value_table);
    RUN(test_valid_config);
    RUN(test_rejects);
    RUN(test_config_text_and_crc);
    RUN(test_slot_frames);
    RUN(test_reading_to_sample);
    RUN(test_acq_grid);
    RUN(test_node_table);
    TEST_MAIN_END();
}
