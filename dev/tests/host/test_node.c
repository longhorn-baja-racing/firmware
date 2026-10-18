#include "lbr/node.h"
#include "lbr/node_table.h"
#include "lbr/nodes.h"
#include "lbr/value_table.h"
#include "lbr_test.h"

static const lbr_signal_id_t k_susp[] = {
    LBR_SIG_SUSP_TRAVEL_FL,
    LBR_SIG_SUSP_TRAVEL_FR,
    LBR_SIG_SUSP_TRAVEL_RL,
    LBR_SIG_SUSP_TRAVEL_RR,
};
static const lbr_signal_id_t k_dup[] = {LBR_SIG_SUSP_TRAVEL_RR};

static lbr_sensor_t s_pots;
static lbr_sensor_t s_dup;

static lbr_node_config_t base_config(lbr_acq_t *acq)
{
    lbr_sensor_unconfigured(&s_pots, "pots", k_susp, 4);
    acq[0]                = (lbr_acq_t){&s_pots, 200, 1};
    lbr_node_config_t cfg = {
        .node_id          = LBR_NODE_SUSPENSION,
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
    CHECK_EQ_I(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err), 0);
    /* 4 ch * 200 Hz + 12 pages/s = 812 frames/s * 160 bit */
    CHECK_EQ_U(lbr_node_can_load_bps(&cfg), 812u * 160u);
}

static void test_rejects(void)
{
    lbr_acq_t         acq[2];
    char              err[128] = "";
    lbr_node_config_t cfg      = base_config(acq);

    cfg.node_id = LBR_NODE_GATEWAY;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    cfg = base_config(acq);

    acq[0].rate_hz = 0;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    acq[0].rate_hz = 2000;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    cfg = base_config(acq);

    /* Decimation without a local log would discard samples silently. */
    acq[0].publish_div = 4;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    CHECK(strstr(err, "local_log") != NULL);
    cfg.local_log = true;
    CHECK_EQ_I(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err), 0);
    cfg = base_config(acq);

    /* Over the per-node CAN budget (4 ch * 1 kHz = 640 kbit/s). */
    acq[0].rate_hz = 1000;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    CHECK(strstr(err, "CAN load") != NULL);
    CHECK_EQ_I(lbr_node_config_validate(&cfg, 1000000, 800, err, sizeof err), 0);
    /* The same plan at 500 kbit/s is over budget even at 200 Hz. */
    acq[0].rate_hz = 200;
    CHECK(lbr_node_config_validate(&cfg, 500000, 150, err, sizeof err) < 0);
    cfg = base_config(acq);

    /* Same signal from two drivers. */
    lbr_sensor_unconfigured(&s_dup, "dup", k_dup, 1);
    acq[1]    = (lbr_acq_t){&s_dup, 10, 1};
    cfg.n_acq = 2;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
    CHECK(strstr(err, "twice") != NULL);
    cfg = base_config(acq);

    cfg.listen_all = true;
    CHECK(lbr_node_config_validate(&cfg, 1000000, 150, err, sizeof err) < 0);
}

static void test_config_text_and_crc(void)
{
    lbr_acq_t         acq[2];
    lbr_node_config_t cfg = base_config(acq);
    char              text[512];
    size_t            n = lbr_node_config_text(&cfg, text, sizeof text);
    CHECK(n > 0 && n < sizeof text);
    CHECK(strstr(text, "node=suspension\n") != NULL);
    CHECK(strstr(text,
                 "sensor.0=pots rate_hz=200 publish_div=1 sim=0 "
                 "signals=SUSP_TRAVEL_FL,SUSP_TRAVEL_FR,SUSP_TRAVEL_RL,SUSP_TRAVEL_RR\n") != NULL);
    uint32_t crc   = lbr_node_config_crc(&cfg);
    acq[0].rate_hz = 100;
    CHECK(lbr_node_config_crc(&cfg) != crc);
    /* Truncation is safe. */
    char small[8];
    CHECK_EQ_U(lbr_node_config_text(&cfg, small, sizeof small), 7u);
    CHECK_EQ_U(small[7], 0);
}

static void test_stream_decimation(void)
{
    lbr_stream_t  st = {0};
    lbr_reading_t r  = {.valid = true, .v.f32 = 3.0f};
    lbr_sample_t  s;
    uint32_t      pub_seq   = 0;
    int           published = 0;
    for (uint32_t i = 0; i < 12; i++) {
        bool pub = lbr_stream_sample(&st, 4, LBR_NODE_STRESS, LBR_SIG_STRAIN_1, &r, 1000 + i, true,
                                     &s, &pub_seq);
        CHECK_EQ_U(s.seq, i); /* local stream: every sample */
        CHECK(s.valid && s.v.f32 == 3.0f && s.type == LBR_VAL_F32);
        if (pub) {
            CHECK_EQ_U(pub_seq, (uint32_t)published); /* CAN stream: contiguous */
            published++;
        }
    }
    CHECK_EQ_I(published, 3);

    lbr_reading_t bad = {.valid = false, .reason = LBR_UNAVAIL_BUS_ERROR};
    lbr_stream_sample(&st, 1, LBR_NODE_STRESS, LBR_SIG_STRAIN_1, &bad, 1, false, &s, &pub_seq);
    CHECK(!s.valid);
    CHECK_EQ_U(s.v.u32, LBR_UNAVAIL_BUS_ERROR);
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

    /* Table full is counted, not silent. */
    for (uint8_t id = 0x20; id < 0x20 + LBR_NODE_TABLE_SIZE; id++) {
        lbr_can_encode_health_page(id, &h, 0, &f);
        lbr_node_table_on_frame(&t, &f, 1);
    }
    CHECK(t.table_full > 0);
}

static void test_value_table(void)
{
    lbr_value_table_t t;
    lbr_value_table_init(&t, 1000000);
    CHECK(lbr_value_table_watch(&t, LBR_SIG_TEMP_CVT, 0));
    CHECK(lbr_value_table_watch(&t, LBR_SIG_FUEL_LEVEL, LBR_NODE_FUEL_GAUGE));
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
    f.source = LBR_NODE_FUEL_GAUGE;
    lbr_value_table_offer(&t, &f, 10);
    CHECK_EQ_I(lbr_value_table_get(&t, LBR_SIG_FUEL_LEVEL, 10, &out), LBR_VALUE_OK);

    for (unsigned i = 2; i < LBR_VALUE_TABLE_MAX; i++) {
        CHECK(lbr_value_table_watch(&t, LBR_SIG_ACCEL_X, 0));
    }
    CHECK(!lbr_value_table_watch(&t, LBR_SIG_ACCEL_Y, 0));
}

int main(void)
{
    RUN(test_value_table);
    RUN(test_valid_config);
    RUN(test_rejects);
    RUN(test_config_text_and_crc);
    RUN(test_stream_decimation);
    RUN(test_node_table);
    TEST_MAIN_END();
}
