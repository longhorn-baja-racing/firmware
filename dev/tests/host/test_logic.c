#include "lbr/bytes.h"
#include "lbr/can.h"
#include "lbr/logic.h"
#include "lbr/nodes.h"
#include "lbr_test.h"

/* Car logic (lbr/logic.h): tasks on a grid, signals with status, outputs as
 * KEY / VALUE log records. */

typedef struct {
    uint8_t  type[64];
    uint8_t  payload[64][64];
    size_t   len[64];
    size_t   n;
    uint32_t overflow;
} sink_t;

static void record(void *ctx, uint8_t type, const uint8_t *payload, size_t len)
{
    sink_t *s = ctx;
    if (s->n >= 64 || len > 64) {
        s->overflow++;
        return;
    }
    s->type[s->n] = type;
    memcpy(s->payload[s->n], payload, len);
    s->len[s->n++] = len;
}

static lbr_sample_t sample(lbr_signal_id_t sig, float v)
{
    return (lbr_sample_t){.signal = sig, .type = LBR_VAL_F32, .valid = true, .v.f32 = v};
}

/* What the test task saw on its last run. */
static lbr_f32  s_seen;
static lbr_bool s_seen_link;
static int      s_runs_fast, s_runs_slow, s_inits;

LBR_OUTPUT(k_ratio, "CVT/Ratio", F32, NONE);
LBR_OUTPUT(k_hot, "Alarms/Hot", BOOL, NONE);
LBR_OUTPUT(k_bad_key, "has space", F32, NONE);

static void fast_init(lbr_cycle *c)
{
    (void)c;
    s_inits++;
}

static void fast_periodic(lbr_cycle *c)
{
    s_runs_fast++;
    s_seen      = lbr_get(c, ENGINE_RPM);
    s_seen_link = lbr_get_bool(c, COCKPIT_LINK);
    lbr_out(c, &k_ratio, lbr_div(lbr_get(c, ENGINE_RPM), lbr_get(c, CVT_RPM)));
}

static void slow_periodic(lbr_cycle *c)
{
    s_runs_slow++;
    lbr_out_bool(c, &k_hot, lbr_gt(lbr_get(c, TEMP_CVT), 100.0f));
    lbr_out(c, &k_bad_key, lbr_f32_of(1.0f)); /* refused: counted, not logged */
}

LBR_TASK(fast, .rate_hz = 50, .init = fast_init, .periodic = fast_periodic);
LBR_TASK(slow, .rate_hz = 10, .periodic = slow_periodic);
static const lbr_task *const k_tasks[] = {&fast, &slow};

static void test_check(void)
{
    char err[96];
    CHECK_EQ_I(lbr_logic_check(k_tasks, 2, err, sizeof err), 0);
    LBR_TASK(odd, .rate_hz = 30, .periodic = slow_periodic);
    const lbr_task *const bad_rate[] = {&odd};
    CHECK(lbr_logic_check(bad_rate, 1, err, sizeof err) != 0);
    CHECK(strstr(err, "must divide") != NULL);
    const lbr_task *const twice[] = {&fast, &fast};
    CHECK(lbr_logic_check(twice, 2, err, sizeof err) != 0);
    CHECK(strstr(err, "two tasks") != NULL);
    LBR_TASK(empty, .rate_hz = 10);
    const lbr_task *const none[] = {&empty};
    CHECK(lbr_logic_check(none, 1, err, sizeof err) != 0);

    lbr_logic_t l;
    lbr_logic_init(&l, bad_rate, 1, (lbr_logic_sink_t){0});
    CHECK(!l.valid);
    CHECK_EQ_U(l.n_tasks, 0); /* an invalid list runs nothing */
}

static void test_schedule_and_outputs(void)
{
    static lbr_logic_t l;
    sink_t             sink = {0};
    lbr_node_table_t   nodes;
    lbr_node_table_init(&nodes, 3000000u);
    lbr_logic_init(&l, k_tasks, 2, (lbr_logic_sink_t){.ctx = &sink, .record = record});
    s_runs_fast = s_runs_slow = s_inits = 0;

    /* The powertrain is on the bus and sends speeds. */
    lbr_can_frame_t hb = {.id = LBR_CAN_ID_HEARTBEAT_BASE + LBR_NODE_POWERTRAIN, .dlc = 2};
    lbr_node_table_on_frame(&nodes, &hb, 1000000u);
    lbr_sample_t e = sample(LBR_SIG_ENGINE_RPM, 3000.0f);
    lbr_sample_t v = sample(LBR_SIG_CVT_RPM, 1500.0f);
    lbr_logic_offer(&l, &e, 1000000u);
    lbr_logic_offer(&l, &v, 1000000u);

    /* One second of 10 ms ticks: 50 + 10 runs, on the grid. */
    for (uint64_t t = 1000000u; t < 2000000u; t += 10000u) {
        lbr_logic_step(&l, t, &nodes);
        if (t == 1000000u) {
            CHECK(lbr_ok(s_seen));
            CHECK(s_seen.v == 3000.0f);
        }
    }
    CHECK_EQ_I(s_inits, 1);
    CHECK_EQ_I(s_runs_fast, 50);
    CHECK_EQ_I(s_runs_slow, 10);
    CHECK_EQ_U(l.late, 0);
    CHECK_EQ_U(l.outputs_refused, 10); /* the bad key, every slow run */

    /* KEY then VALUE records: ratio 2.0 first. */
    CHECK_EQ_U(sink.type[0], LBR_REC_KEY);
    CHECK_EQ_U(lbr_get_u16(sink.payload[0]), 1);
    CHECK_EQ_U(sink.payload[0][2], LBR_VAL_F32);
    CHECK(sink.len[0] == 4 + strlen("CVT/Ratio"));
    CHECK(memcmp(sink.payload[0] + 4, "CVT/Ratio", 9) == 0);
    CHECK_EQ_U(sink.type[1], LBR_REC_VALUE);
    CHECK_EQ_U(sink.len[1], LBR_VALUE_REC_SIZE);
    CHECK_EQ_U(lbr_get_u64(sink.payload[1]), 1000000u);
    CHECK_EQ_U(sink.payload[1][10], LBR_OK);
    CHECK(lbr_bits_f32(lbr_get_u32(sink.payload[1] + 11)) == 2.0f);

    /* The data went quiet (the node is still there): STALE after 3 periods. */
    s_seen = lbr_f32_of(0.0f);
    lbr_logic_step(&l, 2000000u, &nodes);
    CHECK_EQ_U(s_seen.st, LBR_STALE);
    CHECK(s_seen.age_ms >= 999);

    /* The node drops off: NODE_LOST, and the derived output says so. */
    uint8_t lost[4];
    CHECK_EQ_U(lbr_node_table_expire(&nodes, 5000000u, lost, sizeof lost), 1);
    sink.n = 0;
    lbr_logic_step(&l, 5000000u, &nodes);
    CHECK_EQ_U(s_seen.st, LBR_NODE_LOST);
    CHECK_EQ_U(sink.type[0], LBR_REC_VALUE);
    CHECK_EQ_U(sink.payload[0][10], LBR_NODE_LOST);
    CHECK_EQ_U(lbr_get_u32(sink.payload[0] + 11), 0); /* no stale number logged */

    /* Never heard, then reported missing: NO_DATA, then NODE_MISSING. */
    CHECK_EQ_U(lbr_signal(NULL, LBR_SIG_ENGINE_RPM).st, LBR_UNSPECIFIED);
    lbr_logic_step(&l, 5020000u, &nodes);
    CHECK_EQ_U(s_seen_link.st, LBR_NO_DATA); /* the Central's own signal: never "missing" */

    /* Announce re-sends every key. */
    sink.n = 0;
    lbr_logic_announce(&l);
    CHECK_EQ_U(sink.n, 2);
    CHECK_EQ_U(sink.type[0], LBR_REC_KEY);
    CHECK_EQ_U(sink.type[1], LBR_REC_KEY);
    CHECK_EQ_U(sink.overflow, 0);
}

static int  s_missing_runs;
static void missing_periodic(lbr_cycle *c)
{
    s_missing_runs++;
    s_seen = lbr_get(c, TEMP_CVT);
}
LBR_TASK(watch_temp, .rate_hz = 100, .periodic = missing_periodic);

static void test_missing_and_reasons(void)
{
    static lbr_logic_t    l;
    const lbr_task *const tasks[] = {&watch_temp};
    lbr_node_table_t      nodes;
    lbr_node_table_init(&nodes, 1000u);
    lbr_logic_init(&l, tasks, 1, (lbr_logic_sink_t){0});

    lbr_logic_step(&l, 0, &nodes);
    CHECK_EQ_U(s_seen.st, LBR_NO_DATA);
    uint8_t missing[16];
    CHECK(lbr_node_table_missing(&nodes, 0, 2000u, missing, sizeof missing) > 0);
    lbr_logic_step(&l, 10000u, &nodes);
    CHECK_EQ_U(s_seen.st, LBR_NODE_MISSING);

    /* The node comes up and says the part is not fitted. */
    lbr_can_frame_t hb = {.id = LBR_CAN_ID_HEARTBEAT_BASE + LBR_NODE_TEMPERATURE, .dlc = 2};
    lbr_node_table_on_frame(&nodes, &hb, 20000u);
    lbr_sample_t s = {.signal = LBR_SIG_TEMP_CVT, .type = LBR_VAL_F32, .valid = false};
    lbr_sample_set_unavailable(&s, LBR_UNAVAIL_NOT_FITTED);
    lbr_logic_offer(&l, &s, 20000u);
    lbr_logic_step(&l, 20000u, &nodes);
    CHECK_EQ_U(s_seen.st, LBR_NOT_FITTED);
    CHECK_EQ_I(s_missing_runs, 3);

    /* Falling behind is counted, and the task rejoins the grid. */
    lbr_logic_step(&l, 95000u, &nodes);
    CHECK_EQ_U(l.late, 1);
    lbr_logic_step(&l, 100000u, &nodes);
    CHECK_EQ_I(s_missing_runs, 5);
}

/* The Central's logic task wakes on RTOS ticks and reads a microsecond clock:
 * ticks almost never land on the grid exactly, and tasks must still run. */
static void test_off_grid_ticks(void)
{
    static lbr_logic_t l;
    lbr_logic_init(&l, k_tasks, 2, (lbr_logic_sink_t){0});
    s_runs_fast = s_runs_slow = s_inits = 0;
    for (uint64_t t = 1005123u; t <= 2005123u; t += 10000u) {
        lbr_logic_step(&l, t, NULL);
    }
    CHECK_EQ_I(s_inits, 1);
    CHECK_EQ_I(s_runs_fast, 50);
    CHECK_EQ_I(s_runs_slow, 10);
    CHECK_EQ_U(l.late, 0);
}

/* Two outputs with one key would merge into one series: the second is refused. */
LBR_OUTPUT(k_dup_a, "Dup/Key", F32, NONE);
LBR_OUTPUT(k_dup_b, "Dup/Key", F32, NONE);

static void dup_periodic(lbr_cycle *c)
{
    lbr_out(c, &k_dup_a, lbr_f32_of(1.0f));
    lbr_out(c, &k_dup_b, lbr_f32_of(2.0f));
}

LBR_TASK(dup, .rate_hz = 10, .periodic = dup_periodic);

static void test_duplicate_key(void)
{
    static lbr_logic_t           l;
    static const lbr_task *const tasks[] = {&dup};
    lbr_logic_init(&l, tasks, 1, (lbr_logic_sink_t){0});
    lbr_logic_step(&l, 0, NULL);
    CHECK_EQ_U(l.n_outputs, 1);
    CHECK_EQ_U(l.outputs_refused, 1);
    CHECK(lbr_logic_find(&l, "Dup/Key") == &k_dup_a);
}

int main(void)
{
    test_check();
    test_schedule_and_outputs();
    test_missing_and_reasons();
    test_off_grid_ticks();
    test_duplicate_key();
    return lbr_test_failures;
}
