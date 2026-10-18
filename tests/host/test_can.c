#include "lbr/can.h"
#include "lbr/nodes.h"
#include "lbr_test.h"

static void test_id_fields(void)
{
    uint32_t id = lbr_can_id_make(5, LBR_CAN_KIND_SAMPLE, LBR_NODE_STRESS, 0x0903);
    CHECK(id <= 0x1FFFFFFFu);
    lbr_can_id_t f = lbr_can_id_parse(id);
    CHECK_EQ_U(f.prio, 5);
    CHECK_EQ_U(f.kind, LBR_CAN_KIND_SAMPLE);
    CHECK_EQ_U(f.source, LBR_NODE_STRESS);
    CHECK_EQ_U(f.sub, 0x0903);
    /* Lower priority value => lower id => wins arbitration. */
    CHECK(lbr_can_id_make(0, 0, 0xFF, 0xFFFF) < lbr_can_id_make(1, 0, 0x01, 0x0000));
    CHECK_EQ_U(lbr_can_frame_bits_worst(8), 160u);
}

static void test_sample_frame_roundtrip(void)
{
    lbr_sample_t s = {
        .signal    = LBR_SIG_SUSP_TRAVEL_FL,
        .source    = LBR_NODE_SUSPENSION,
        .type      = LBR_VAL_F32,
        .valid     = true,
        .synced    = true,
        .seq       = 130, /* low 6 bits = 2 */
        .t_mono_us = 5000000123ull,
        .v.f32     = 42.5f,
    };
    lbr_can_frame_t f;
    CHECK(lbr_can_encode_sample(&s, &f));
    CHECK_EQ_U(f.dlc, 8);
    lbr_can_id_t id = lbr_can_id_parse(f.id);
    CHECK_EQ_U(id.prio, lbr_signal_info(LBR_SIG_SUSP_TRAVEL_FL)->can_prio);
    CHECK_EQ_U(id.sub, LBR_SIG_SUSP_TRAVEL_FL);

    /* Received 1.5 ms later on the gateway clock. */
    lbr_sample_t o;
    CHECK(lbr_can_decode_sample(&f, s.t_mono_us + 1500, &o));
    CHECK_EQ_U(o.t_mono_us, s.t_mono_us);
    CHECK_EQ_U(o.seq, 2u);
    CHECK(o.seq_short);
    CHECK(o.valid && o.synced);
    CHECK_EQ_U(o.type, LBR_VAL_F32);
    CHECK(o.v.f32 == 42.5f);
    CHECK_EQ_U(o.source, LBR_NODE_SUSPENSION);

    /* Unavailable with reason. */
    lbr_sample_set_unavailable(&s, LBR_UNAVAIL_SENSOR_FAULT);
    s.synced = false;
    CHECK(lbr_can_encode_sample(&s, &f));
    CHECK(lbr_can_decode_sample(&f, s.t_mono_us, &o));
    CHECK(!o.valid && !o.synced);
    CHECK_EQ_U(o.v.u32, LBR_UNAVAIL_SENSOR_FAULT);
}

static void test_unwrap(void)
{
    const uint64_t T = 1ull << 24;
    /* Sample just before a 24-bit wrap, received just after. */
    uint64_t t = 10 * T - 100;
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(t & 0xFFFFFF), 10 * T + 50), t);
    /* Receiver slightly behind the stamp (clock jitter): still exact. */
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(t & 0xFFFFFF), t - 20), t);
    /* Up to just under half the range either way. */
    uint64_t far = 7 * T + 12345;
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(far & 0xFFFFFF), far + T / 2 - 1), far);
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(far & 0xFFFFFF), far - T / 2 + 1), far);
    /* Never negative near boot. */
    CHECK_EQ_U(lbr_can_unwrap_time(0xFFFFF0, 5), 0u);
}

static void test_unknown_signal(void)
{
    lbr_can_frame_t f = {
        .id   = lbr_can_id_make(4, LBR_CAN_KIND_SAMPLE, 0x42, 0x7ABC),
        .dlc  = 8,
        .data = {0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0x80},
    };
    lbr_sample_t o;
    CHECK(lbr_can_decode_sample(&f, 0, &o));
    CHECK_EQ_U(o.type, LBR_VAL_RAW);
    CHECK_EQ_U(o.v.u32, 0x12345678u);
    /* Encoder refuses ids outside the catalog. */
    lbr_sample_t bad = {.signal = (lbr_signal_id_t)0x7ABC, .source = 2, .type = LBR_VAL_U32};
    CHECK(!lbr_can_encode_sample(&bad, &f));
    /* Wrong kind / dlc are not samples. */
    f.dlc = 7;
    CHECK(!lbr_can_decode_sample(&f, 0, &o));
}

static void test_time_sync_frames(void)
{
    lbr_can_frame_t f;
    lbr_can_encode_time_sync(LBR_NODE_GATEWAY, 200, 0xBEEF, &f);
    uint8_t  seq;
    uint16_t epoch;
    CHECK(lbr_can_decode_time_sync(&f, &seq, &epoch));
    CHECK_EQ_U(seq, 200);
    CHECK_EQ_U(epoch, 0xBEEF);
    CHECK_EQ_U(lbr_can_id_parse(f.id).prio, LBR_CAN_PRIO_TIME);

    uint64_t tx = 0x00ABCDEF01234567ull; /* 56 bits */
    lbr_can_encode_time_follow_up(LBR_NODE_GATEWAY, 201, tx, &f);
    uint64_t got;
    CHECK(lbr_can_decode_time_follow_up(&f, &seq, &got));
    CHECK_EQ_U(seq, 201);
    CHECK_EQ_U(got, tx);
    CHECK(!lbr_can_decode_time_sync(&f, &seq, &epoch));
}

static void test_health_frames(void)
{
    lbr_health_record_t h = {.uptime_ms = 1234, .state = LBR_NODE_RUN, .can_bus_off_count = 3};
    lbr_health_record_t o = {0};
    lbr_can_frame_t     f;
    CHECK(lbr_can_encode_health_page(LBR_NODE_FUEL_GAUGE, &h, 0, &f));
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), 0);
    CHECK_EQ_U(o.node_id, LBR_NODE_FUEL_GAUGE);
    CHECK_EQ_U(o.uptime_ms, 1234u);
    CHECK(lbr_can_encode_health_page(LBR_NODE_FUEL_GAUGE, &h, 3, &f));
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), 3);
    CHECK_EQ_U(o.can_bus_off_count, 3u);
    CHECK(!lbr_can_encode_health_page(1, &h, LBR_HEALTH_PAGE_COUNT, &f));
    lbr_can_encode_time_sync(1, 0, 0, &f);
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), -1);
}

static void test_frame_record(void)
{
    lbr_can_frame_t f = {.id = 0x1ABCDEF, .dlc = 3, .data = {1, 2, 3, 9, 9, 9, 9, 9}};
    uint8_t         buf[LBR_CAN_FRAME_REC_SIZE];
    CHECK_EQ_U(lbr_can_frame_rec_encode(777, LBR_CAN_REC_FLAG_EXT, &f, buf, sizeof buf),
               LBR_CAN_FRAME_REC_SIZE);
    CHECK_EQ_U(buf[14 + 3], 0); /* padding zeroed: deterministic logs */
    uint64_t        t;
    uint8_t         flags;
    lbr_can_frame_t o;
    CHECK_EQ_U(lbr_can_frame_rec_decode(buf, sizeof buf, &t, &flags, &o), LBR_CAN_FRAME_REC_SIZE);
    CHECK_EQ_U(t, 777u);
    CHECK_EQ_U(flags, LBR_CAN_REC_FLAG_EXT);
    CHECK_EQ_U(o.id, f.id);
    CHECK_EQ_U(o.dlc, 3);
    CHECK(memcmp(o.data, f.data, 3) == 0);
    buf[13] = 9;
    CHECK_EQ_U(lbr_can_frame_rec_decode(buf, sizeof buf, &t, &flags, &o), 0u);
}

int main(void)
{
    RUN(test_id_fields);
    RUN(test_sample_frame_roundtrip);
    RUN(test_unwrap);
    RUN(test_unknown_signal);
    RUN(test_time_sync_frames);
    RUN(test_health_frames);
    RUN(test_frame_record);
    TEST_MAIN_END();
}
