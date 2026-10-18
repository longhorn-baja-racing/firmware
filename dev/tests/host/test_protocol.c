#include "lbr/bytes.h"
#include "lbr/nodes.h"
#include "lbr/protocol.h"
#include "lbr_test.h"

static void test_catalog_integrity(void)
{
    size_t n = lbr_signal_count();
    CHECK(n > 0);
    for (size_t i = 0; i < n; i++) {
        const lbr_signal_info_t *s = lbr_signal_at(i);
        CHECK(s->id != LBR_SIG_UNKNOWN);
        CHECK(s->type > LBR_VAL_NONE && s->type < LBR_VAL_RAW);
        CHECK((unsigned)s->unit < LBR_UNIT_COUNT_);
        CHECK(s->can_prio <= 7);
        CHECK(s->name != NULL && s->name[0] != '\0');
        CHECK(s->desc != NULL && s->desc[0] != '\0');
        CHECK(lbr_signal_info(s->id) == s);
        if (i > 0) {
            /* Sorted strictly ascending: unique ids and binary search valid. */
            CHECK(lbr_signal_at(i - 1)->id < s->id);
        }
        for (size_t j = 0; j < i; j++) {
            CHECK(strcmp(lbr_signal_at(j)->name, s->name) != 0);
        }
        for (const char *c = s->desc; *c; c++) {
            CHECK((unsigned char)*c < 0x80); /* ASCII metadata */
        }
    }
    CHECK(lbr_signal_info((lbr_signal_id_t)0x7FFF) == NULL);
    CHECK(lbr_signal_at(n) == NULL);
}

static void test_ids_stable(void)
{
    /* Ids are wire protocol: these must never change. */
    CHECK_EQ_U(LBR_SIG_ACCEL_X, 0x0100);
    CHECK_EQ_U(LBR_SIG_IMU_CALIB_STATE, 0x010B);
    CHECK_EQ_U(LBR_SIG_ENGINE_RPM, 0x0200);
    CHECK_EQ_U(LBR_SIG_FUEL_LEVEL, 0x0206);
    CHECK_EQ_U(LBR_SIG_TEMP_AMBIENT, 0x0304);
    CHECK_EQ_U(LBR_SIG_BATT_CURRENT, 0x0401);
    CHECK_EQ_U(LBR_SIG_EVENT, 0x0700);
    CHECK_EQ_U(LBR_SIG_FAULT, 0x0701);
    CHECK_EQ_U(LBR_NODE_GATEWAY, 0x01);
    CHECK_EQ_U(LBR_NODE_DASHBOARD, 0x08);
    CHECK(strcmp(lbr_node_name(LBR_NODE_IMU_GPS), "imu_gps") == 0);
    CHECK(strcmp(lbr_node_name(0x77), "?") == 0);
}

static lbr_sample_t roundtrip(const lbr_sample_t *s)
{
    uint8_t      buf[LBR_SAMPLE_WIRE_SIZE];
    lbr_sample_t out;
    memset(&out, 0xCC, sizeof out);
    CHECK_EQ_U(lbr_sample_encode(s, buf, sizeof buf), LBR_SAMPLE_WIRE_SIZE);
    CHECK_EQ_U(lbr_sample_decode(buf, sizeof buf, &out), LBR_SAMPLE_WIRE_SIZE);
    CHECK_EQ_U(out.signal, s->signal);
    CHECK_EQ_U(out.source, s->source);
    CHECK_EQ_U(out.type, s->type);
    CHECK(out.valid == s->valid);
    CHECK(out.synced == s->synced);
    CHECK(out.seq_short == s->seq_short);
    CHECK_EQ_U(out.seq, s->seq);
    CHECK_EQ_U(out.t_mono_us, s->t_mono_us);
    CHECK_EQ_U(lbr_sample_value_bits(&out), lbr_sample_value_bits(s));
    return out;
}

static void test_sample_roundtrip(void)
{
    lbr_sample_t s = {
        .signal    = LBR_SIG_TEMP_CVT,
        .source    = LBR_NODE_TEMPERATURE,
        .type      = LBR_VAL_F32,
        .valid     = true,
        .synced    = true,
        .seq       = 0xFFFFFFFFu,
        .t_mono_us = 0x0123456789ABCDEFull,
        .v.f32     = -40.25f,
    };
    lbr_sample_t o = roundtrip(&s);
    CHECK(o.v.f32 == -40.25f);

    s.type  = LBR_VAL_I32;
    s.v.i32 = -123456789;
    o       = roundtrip(&s);
    CHECK_EQ_I(o.v.i32, -123456789);

    s.type = LBR_VAL_BOOL;
    s.v.b  = true;
    o      = roundtrip(&s);
    CHECK(o.v.b);

    s.type      = LBR_VAL_STATUS;
    s.v.status  = 0xDEADBEEF;
    s.seq_short = true;
    o           = roundtrip(&s);
    CHECK_EQ_U(o.v.status, 0xDEADBEEFu);

    /* Unavailable: reason travels in the value slot, never a fake zero. */
    s.type = LBR_VAL_F32;
    lbr_sample_set_unavailable(&s, LBR_UNAVAIL_NOT_FITTED);
    o = roundtrip(&s);
    CHECK(!o.valid);
    CHECK_EQ_U(o.v.u32, LBR_UNAVAIL_NOT_FITTED);
}

static void test_sample_layout(void)
{
    lbr_sample_t s = {
        .signal    = (lbr_signal_id_t)0x0302,
        .source    = 0x05,
        .type      = LBR_VAL_F32,
        .valid     = true,
        .synced    = true,
        .seq       = 7,
        .t_mono_us = 1000,
        .v.f32     = 1.0f,
    };
    uint8_t buf[LBR_SAMPLE_WIRE_SIZE];
    lbr_sample_encode(&s, buf, sizeof buf);
    const uint8_t expect[LBR_SAMPLE_WIRE_SIZE] = {
        0x02, 0x03, 0x05, 0x31, 0x07, 0x00, 0x00, 0x00, 0xE8, 0x03,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
    };
    CHECK(memcmp(buf, expect, sizeof expect) == 0);
}

static void test_sample_reject(void)
{
    lbr_sample_t s = {.signal = LBR_SIG_ACCEL_X, .source = 2, .type = LBR_VAL_F32, .valid = true};
    uint8_t      buf[LBR_SAMPLE_WIRE_SIZE];
    lbr_sample_t out;
    CHECK_EQ_U(lbr_sample_encode(&s, buf, sizeof buf - 1), 0u);
    CHECK_EQ_U(lbr_sample_encode(&s, buf, sizeof buf), LBR_SAMPLE_WIRE_SIZE);
    CHECK_EQ_U(lbr_sample_decode(buf, sizeof buf - 1, &out), 0u);
    buf[3] |= 0x80; /* reserved bit */
    CHECK_EQ_U(lbr_sample_decode(buf, sizeof buf, &out), 0u);
    buf[3] = 0x0F; /* unknown value type */
    CHECK_EQ_U(lbr_sample_decode(buf, sizeof buf, &out), 0u);
    s.type = LBR_VAL_COUNT_;
    CHECK_EQ_U(lbr_sample_encode(&s, buf, sizeof buf), 0u);
}

static void test_seq_rollover(void)
{
    CHECK_EQ_I(lbr_seq_delta(5, 6), 1);
    CHECK_EQ_I(lbr_seq_delta(6, 6), 0);
    CHECK_EQ_I(lbr_seq_delta(0xFFFFFFFFu, 0), 1);
    CHECK_EQ_I(lbr_seq_delta(0xFFFFFFFEu, 3), 5);
    CHECK_EQ_I(lbr_seq_delta(3, 0xFFFFFFFEu), -5);
    CHECK_EQ_I(lbr_seq_delta_bits(63, 0, 6), 1);
    CHECK_EQ_I(lbr_seq_delta_bits(60, 2, 6), 6);
    CHECK_EQ_I(lbr_seq_delta_bits(2, 60, 6), -6);
    CHECK_EQ_I(lbr_seq_delta_bits(10, 10, 6), 0);
}

static void test_catalog_hash(void)
{
    uint32_t h = lbr_catalog_hash();
    CHECK(h != 0);
    CHECK_EQ_U(h, lbr_catalog_hash());
}

static void test_unit_names(void)
{
    for (unsigned u = 0; u < LBR_UNIT_COUNT_; u++) {
        CHECK(lbr_unit_name((lbr_unit_t)u) != NULL);
    }
    CHECK(strcmp(lbr_unit_name(LBR_UNIT_DEG_C), "degC") == 0);
    CHECK(strcmp(lbr_value_type_name(LBR_VAL_STATUS), "status") == 0);
}

int main(void)
{
    RUN(test_catalog_integrity);
    RUN(test_ids_stable);
    RUN(test_sample_roundtrip);
    RUN(test_sample_layout);
    RUN(test_sample_reject);
    RUN(test_seq_rollover);
    RUN(test_catalog_hash);
    RUN(test_unit_names);
    TEST_MAIN_END();
}
