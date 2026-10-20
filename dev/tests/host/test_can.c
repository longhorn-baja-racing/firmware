#include <math.h>

#include "lbr/bytes.h"
#include "lbr/can.h"
#include "lbr/nodes.h"
#include "lbr_test.h"

static lbr_sample_t f32_sample(lbr_signal_id_t sig, float v)
{
    lbr_sample_t s = {.signal = sig, .type = LBR_VAL_F32, .valid = true};
    s.v.f32        = v;
    return s;
}

static void test_sender_map(void)
{
    CHECK_EQ_U(lbr_can_sender(LBR_CAN_ID_TIME_SYNC), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(LBR_CAN_ID_TIME_FOLLOW_UP), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(LBR_CAN_ID_COMMAND), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(0x023), LBR_NODE_DRIVER_INPUT);
    CHECK_EQ_U(lbr_can_sender(0x100), LBR_NODE_DRIVER_INPUT);
    CHECK_EQ_U(lbr_can_sender(0x201), LBR_NODE_POWERTRAIN);
    CHECK_EQ_U(lbr_can_sender(0x304), LBR_NODE_IMU); /* BL accel: the IMU Controller sends it */
    CHECK_EQ_U(lbr_can_sender(0x401), LBR_NODE_STRESS);
    CHECK_EQ_U(lbr_can_sender(0x501), LBR_NODE_TEMPERATURE);
    CHECK_EQ_U(lbr_can_sender(0x60A), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(0x701), LBR_NODE_IMU);
    CHECK_EQ_U(lbr_can_sender(0x706), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(0x780 + 16 * 5 + 11), LBR_NODE_STRESS);
    /* Unassigned ids, page 12+, and anything above 11 bits are not LBR. */
    CHECK_EQ_U(lbr_can_sender(0x000), 0u);
    CHECK_EQ_U(lbr_can_sender(0x020), 0u);
    CHECK_EQ_U(lbr_can_sender(0x027), 0u);
    CHECK_EQ_U(lbr_can_sender(0x707), 0u);
    CHECK_EQ_U(lbr_can_sender(0x780 + 16 * 2 + 12), 0u);
    CHECK_EQ_U(lbr_can_sender(0x7FF), 0u);
    CHECK_EQ_U(lbr_can_sender(0x1ABCDEF), 0u);
    /* Worst-case standard frame: 34 + 64 bits, 24 stuff bits, 13 trailer. */
    CHECK_EQ_U(lbr_can_frame_bits_worst(8), 135u);
    CHECK_EQ_U(lbr_can_frame_bits_worst(0), 55u);
}

static void test_field_codec(void)
{
    lbr_field_def_t i16 = {
        .signal = LBR_SIG_TEMP_CVT, .offset = 2, .enc = LBR_ENC_I16, .scale = 0.1f};
    uint8_t      p[8] = {0};
    lbr_sample_t s    = f32_sample(LBR_SIG_TEMP_CVT, 87.46f);
    lbr_sample_t o;

    CHECK(lbr_can_field_encode(&i16, &s, p));
    CHECK_EQ_I((int16_t)lbr_get_u16(p + 2), 875); /* rounded to the nearest LSB */
    CHECK_EQ_U(p[0], 0);
    lbr_can_field_decode(&i16, p, &o);
    CHECK(o.valid && o.type == LBR_VAL_F32);
    CHECK(fabsf(o.v.f32 - 87.5f) < 1e-4f);

    /* Negative values and the edge of the valid range. */
    s.v.f32 = -3276.0f;
    CHECK(lbr_can_field_encode(&i16, &s, p));
    lbr_can_field_decode(&i16, p, &o);
    CHECK(o.valid && fabsf(o.v.f32 + 3276.0f) < 1e-3f);

    /* Too big, too small (into the reserved band), NaN: OUT_OF_RANGE, not a number. */
    float bad[] = {3300.0f, -3276.5f, NAN, INFINITY};
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        s.v.f32 = bad[i];
        CHECK(!lbr_can_field_encode(&i16, &s, p));
        lbr_can_field_decode(&i16, p, &o);
        CHECK(!o.valid);
        CHECK_EQ_U(o.v.u32, LBR_UNAVAIL_OUT_OF_RANGE);
    }

    /* Every reason survives every encoding. */
    for (unsigned enc = 0; enc < LBR_ENC_COUNT_; enc++) {
        lbr_field_def_t fd = {
            .signal = LBR_SIG_TEMP_CVT, .offset = 0, .enc = (uint8_t)enc, .scale = 1.0f};
        for (unsigned why = 0; why <= LBR_UNAVAIL_BUS_ERROR; why++) {
            lbr_sample_t u = {.signal = LBR_SIG_TEMP_CVT, .type = LBR_VAL_F32};
            lbr_sample_set_unavailable(&u, (lbr_unavail_reason_t)why);
            memset(p, 0, sizeof p);
            CHECK(lbr_can_field_encode(&fd, &u, p));
            lbr_can_field_decode(&fd, p, &o);
            CHECK(!o.valid);
            CHECK_EQ_U(o.v.u32, why);
        }
    }

    /* Unsigned: zero is a value, and the top 8 raw values are reserved. */
    lbr_field_def_t u16 = {
        .signal = LBR_SIG_ENGINE_RPM, .offset = 0, .enc = LBR_ENC_U16, .scale = 1.0f};
    s = f32_sample(LBR_SIG_ENGINE_RPM, 0.0f);
    CHECK(lbr_can_field_encode(&u16, &s, p));
    lbr_can_field_decode(&u16, p, &o);
    CHECK(o.valid && o.v.f32 == 0.0f);
    s.v.f32 = 65527.0f;
    CHECK(lbr_can_field_encode(&u16, &s, p));
    s.v.f32 = 65528.0f;
    CHECK(!lbr_can_field_encode(&u16, &s, p));
    s.v.f32 = -1.0f;
    CHECK(!lbr_can_field_encode(&u16, &s, p));

    /* Integer signals travel unscaled. */
    lbr_field_def_t i32 = {
        .signal = LBR_SIG_GPS_LATITUDE, .offset = 4, .enc = LBR_ENC_I32, .scale = 1.0f};
    lbr_sample_t lat = {.signal = LBR_SIG_GPS_LATITUDE, .type = LBR_VAL_I32, .valid = true};
    lat.v.i32        = -973456789;
    CHECK(lbr_can_field_encode(&i32, &lat, p));
    lbr_can_field_decode(&i32, p, &o);
    CHECK(o.valid && o.type == LBR_VAL_I32 && o.v.i32 == -973456789);
    lbr_field_def_t u8 = {
        .signal = LBR_SIG_DASH_LINK, .offset = 1, .enc = LBR_ENC_U8, .scale = 1.0f};
    lbr_sample_t b = {.signal = LBR_SIG_DASH_LINK, .type = LBR_VAL_BOOL, .valid = true};
    b.v.b          = true;
    CHECK(lbr_can_field_encode(&u8, &b, p));
    CHECK_EQ_U(p[1], 1);
    lbr_can_field_decode(&u8, p, &o);
    CHECK(o.valid && o.type == LBR_VAL_BOOL && o.v.b);
}

static void test_grid_index(void)
{
    const uint32_t P = 10000; /* 100 Hz */
    /* Exact, a little late, a little early (receiver clock behind). */
    CHECK_EQ_U(lbr_can_grid_index(1234, 1234ull * P + 300, P), 1234u);
    CHECK_EQ_U(lbr_can_grid_index(1234, 1236ull * P, P), 1234u);
    CHECK_EQ_U(lbr_can_grid_index(1234, 1233ull * P + 9000, P), 1234u);
    /* Across the 16-bit wrap. */
    uint64_t k = 5ull * 65536 + 65535;
    CHECK_EQ_U(lbr_can_grid_index((uint16_t)k, (k + 3) * P, P), k);
    k = 5ull * 65536 + 2;
    CHECK_EQ_U(lbr_can_grid_index((uint16_t)k, (k + 10) * P, P), k);
    /* Never negative near boot. */
    CHECK_EQ_U(lbr_can_grid_index(65530, 3 * P, P), 65530u);
}

static void test_data_frame_roundtrip(void)
{
    const lbr_frame_def_t *d = lbr_frame_def(0x100);
    CHECK(d != NULL && d->sender == LBR_NODE_DRIVER_INPUT && d->counter_offset == 6);
    lbr_sample_t v[3] = {
        f32_sample(LBR_SIG_BRAKE_PEDAL, 12.34f),
        f32_sample(LBR_SIG_GAS_PEDAL, 99.99f),
        f32_sample(LBR_SIG_STEERING_ANGLE, -45.5f),
    };
    lbr_sample_set_unavailable(&v[1], LBR_UNAVAIL_NOT_FITTED);
    uint64_t        k = 3ull * 65536 + 777;
    lbr_can_frame_t f;
    CHECK_EQ_U(lbr_can_encode_data(d, v, k, &f), 0u);
    CHECK_EQ_U(f.id, 0x100);
    CHECK_EQ_U(f.dlc, 8);
    CHECK_EQ_U(lbr_get_u16(f.data + 6), 777);

    /* Received 2.5 ms after the grid point. */
    lbr_sample_t o[8];
    CHECK_EQ_U(lbr_can_decode_data(&f, k * 10000 + 2500, true, o, 8), 3u);
    CHECK_EQ_U(o[0].signal, LBR_SIG_BRAKE_PEDAL);
    CHECK(o[0].valid && fabsf(o[0].v.f32 - 12.34f) < 0.006f);
    CHECK(!o[1].valid);
    CHECK_EQ_U(o[1].v.u32, LBR_UNAVAIL_NOT_FITTED);
    CHECK(o[2].valid && fabsf(o[2].v.f32 + 45.5f) < 0.06f);
    for (int i = 0; i < 3; i++) {
        CHECK_EQ_U(o[i].t_mono_us, k * 10000);
        CHECK_EQ_U(o[i].seq, (uint32_t)k);
        CHECK_EQ_U(o[i].source, LBR_NODE_DRIVER_INPUT);
        CHECK(o[i].synced && !o[i].seq_inferred);
    }

    /* Forwarded corner data: the origin is the corner, not the sender. */
    const lbr_frame_def_t *fl = lbr_frame_def(0x301);
    CHECK(fl != NULL && fl->origin == LBR_NODE_CORNER_FL && fl->sender == LBR_NODE_IMU);
    lbr_sample_t g[3] = {f32_sample(LBR_SIG_IMU_FL_GYRO_X, 1.0f),
                         f32_sample(LBR_SIG_IMU_FL_GYRO_Y, -2.0f),
                         f32_sample(LBR_SIG_IMU_FL_GYRO_Z, 40.0f)};
    CHECK_EQ_U(lbr_can_encode_data(fl, g, 5, &f), 1u); /* 40 rad/s does not fit */
    CHECK_EQ_U(lbr_can_decode_data(&f, 50000, false, o, 8), 3u);
    CHECK_EQ_U(o[0].source, LBR_NODE_CORNER_FL);
    CHECK(!o[0].synced);
    CHECK(!o[2].valid && o[2].v.u32 == LBR_UNAVAIL_OUT_OF_RANGE);

    /* Wrong dlc, unknown id, or too little room. */
    f.dlc = 7;
    CHECK_EQ_U(lbr_can_decode_data(&f, 0, true, o, 8), 0u);
    f.dlc = 8;
    f.id  = 0x7F0;
    CHECK_EQ_U(lbr_can_decode_data(&f, 0, true, o, 8), 0u);
    lbr_can_encode_data(d, v, k, &f);
    CHECK_EQ_U(lbr_can_decode_data(&f, k * 10000, true, o, 2), 2u);
}

static void test_no_counter_frame(void)
{
    const lbr_frame_def_t *d = lbr_frame_def(0x400);
    CHECK(d != NULL && d->counter_offset == LBR_FRAME_NO_COUNTER);
    lbr_sample_t v[4] = {f32_sample(LBR_SIG_STRESS_1, 100.0f), f32_sample(LBR_SIG_STRESS_2, -5.0f),
                         f32_sample(LBR_SIG_STRESS_3, 0.0f),
                         f32_sample(LBR_SIG_STRESS_4, 32000.0f)};
    lbr_can_frame_t f;
    CHECK_EQ_U(lbr_can_encode_data(d, v, 42, &f), 0u);
    lbr_sample_t o[4];
    /* Stamped at the last grid point at or before reception. */
    CHECK_EQ_U(lbr_can_decode_data(&f, 420000 + 900, true, o, 4), 4u);
    CHECK_EQ_U(o[0].t_mono_us, 420000u);
    CHECK_EQ_U(o[3].seq, 42u);
    CHECK(o[0].seq_inferred);
    CHECK(o[3].valid && o[3].v.f32 == 32000.0f);
}

static void test_event_frame(void)
{
    lbr_sample_t s = {
        .signal    = LBR_SIG_FAULT,
        .source    = LBR_NODE_TEMPERATURE,
        .type      = LBR_VAL_STATUS,
        .valid     = true,
        .synced    = true,
        .seq       = 65, /* low 6 bits = 1 */
        .t_mono_us = (5ull << 24) + 1234,
    };
    s.v.u32 = lbr_event_value(LBR_EVT_SENSOR_INIT_FAIL, 3);
    lbr_can_frame_t f;
    CHECK(lbr_can_encode_event(&s, &f));
    CHECK_EQ_U(f.id, 0x024);
    lbr_sample_t o;
    CHECK(lbr_can_decode_event(&f, s.t_mono_us + 5000, &o));
    CHECK_EQ_U(o.signal, LBR_SIG_FAULT);
    CHECK_EQ_U(o.source, LBR_NODE_TEMPERATURE);
    CHECK_EQ_U(o.t_mono_us, s.t_mono_us);
    CHECK_EQ_U(o.seq, 1u);
    CHECK(o.synced && o.valid);
    CHECK_EQ_U(o.v.u32 & 0xFFFF, LBR_EVT_SENSOR_INIT_FAIL);
    CHECK_EQ_U(o.v.u32 >> 16, 3u);
    /* Only CAN nodes send events; data frames are not events. */
    s.source = LBR_NODE_CORNER_FL;
    CHECK(!lbr_can_encode_event(&s, &f));
    f.id = 0x100;
    CHECK(!lbr_can_decode_event(&f, 0, &o));
}

static void test_unwrap(void)
{
    const uint64_t T = 1ull << 24;
    uint64_t       t = 10 * T - 100;
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(t & 0xFFFFFF), 10 * T + 50), t);
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(t & 0xFFFFFF), t - 20), t);
    uint64_t far = 7 * T + 12345;
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(far & 0xFFFFFF), far + T / 2 - 1), far);
    CHECK_EQ_U(lbr_can_unwrap_time((uint32_t)(far & 0xFFFFFF), far - T / 2 + 1), far);
    CHECK_EQ_U(lbr_can_unwrap_time(0xFFFFF0, 5), 0u);
    /* Saturates at the top of the range too, instead of wrapping to 0. */
    CHECK_EQ_U(lbr_can_unwrap_time(0x000010, UINT64_MAX - 5), UINT64_MAX);
}

static void test_time_sync_frames(void)
{
    lbr_can_frame_t f;
    lbr_can_encode_time_sync(200, 0xBEEF, 123456789ull, &f);
    CHECK_EQ_U(f.id, 0x001);
    CHECK_EQ_U(f.dlc, 8);
    CHECK_EQ_U(lbr_get_u32(f.data), 123456u); /* ms since the Central booted */
    uint8_t  seq;
    uint16_t epoch;
    CHECK(lbr_can_decode_time_sync(&f, &seq, &epoch));
    CHECK_EQ_U(seq, 200);
    CHECK_EQ_U(epoch, 0xBEEF);

    uint64_t tx = 0x00ABCDEF01234567ull; /* 56 bits */
    lbr_can_encode_time_follow_up(201, tx, &f);
    CHECK_EQ_U(f.id, 0x002);
    uint64_t got;
    CHECK(lbr_can_decode_time_follow_up(&f, &seq, &got));
    CHECK_EQ_U(seq, 201);
    CHECK_EQ_U(got, tx);
    CHECK(!lbr_can_decode_time_sync(&f, &seq, &epoch));
}

static void test_command_frame(void)
{
    uint8_t         args[7] = {1, 2, 3, 4, 5, 6, 7};
    uint8_t         got[7];
    uint8_t         code;
    lbr_can_frame_t f;
    lbr_can_encode_command(LBR_CMD_MARK, args, &f);
    CHECK_EQ_U(f.id, 0x010);
    CHECK(lbr_can_decode_command(&f, &code, got));
    CHECK_EQ_U(code, LBR_CMD_MARK);
    CHECK(memcmp(args, got, 7) == 0);
    f.dlc = 7;
    CHECK(!lbr_can_decode_command(&f, &code, got));
}

static void test_heartbeat_and_health(void)
{
    lbr_can_frame_t f;
    uint8_t         state, flags;
    CHECK(lbr_can_encode_heartbeat(LBR_NODE_STRESS, LBR_NODE_DEGRADED, LBR_HB_ERR_SENSOR, &f));
    CHECK_EQ_U(f.id, 0x705);
    CHECK_EQ_U(f.dlc, 2);
    CHECK_EQ_U(lbr_can_decode_heartbeat(&f, &state, &flags), LBR_NODE_STRESS);
    CHECK_EQ_U(state, LBR_NODE_DEGRADED);
    CHECK_EQ_U(flags, LBR_HB_ERR_SENSOR);
    CHECK(!lbr_can_encode_heartbeat(LBR_NODE_DASHBOARD, 0, 0, &f));

    lbr_health_record_t h = {.uptime_ms = 1234, .state = LBR_NODE_RUN, .can_bus_off_count = 3};
    lbr_health_record_t o = {0};
    CHECK(lbr_can_encode_health_page(LBR_NODE_POWERTRAIN, &h, 0, &f));
    CHECK_EQ_U(f.id, 0x7A0);
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), 0);
    CHECK_EQ_U(o.node_id, LBR_NODE_POWERTRAIN);
    CHECK_EQ_U(o.uptime_ms, 1234u);
    CHECK(lbr_can_encode_health_page(LBR_NODE_POWERTRAIN, &h, 3, &f));
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), 3);
    CHECK_EQ_U(o.can_bus_off_count, 3u);
    CHECK(!lbr_can_encode_health_page(1, &h, LBR_HEALTH_PAGE_COUNT, &f));
    lbr_can_encode_heartbeat(LBR_NODE_IMU, 0, 0, &f);
    CHECK_EQ_I(lbr_can_decode_health_page(&f, &o), -1);
}

static void test_bus_decoder(void)
{
    lbr_bus_decoder_t d;
    lbr_bus_decoder_init(&d);
    lbr_sample_t    out[8];
    lbr_can_frame_t f, hb;

    /* Nothing heard from the node yet: its samples are not on the shared base. */
    lbr_sample_t v[2] = {f32_sample(LBR_SIG_ENGINE_RPM, 3000), f32_sample(LBR_SIG_CVT_RPM, 1500)};
    lbr_can_encode_data(lbr_frame_def(0x200), v, 10, &f);
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 100000, out, 8), 2u);
    CHECK(!out[0].synced);

    lbr_can_encode_heartbeat(LBR_NODE_POWERTRAIN, LBR_NODE_RUN, 0, &hb);
    CHECK_EQ_U(lbr_bus_decode(&d, &hb, 100000, out, 8), 0u);
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 100000, out, 8), 2u);
    CHECK(out[1].synced);
    lbr_can_encode_heartbeat(LBR_NODE_POWERTRAIN, LBR_NODE_DEGRADED, LBR_HB_ERR_NOT_SYNCED, &hb);
    lbr_bus_decode(&d, &hb, 100000, out, 8);
    lbr_bus_decode(&d, &f, 100000, out, 8);
    CHECK(!out[0].synced);

    /* The Central's own data is always on the shared base. */
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_CENTRAL));

    /* Corners: the IMU Controller synced, and the corner's CORNER_SYNCED bit. */
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_FR));
    lbr_sample_t st[6];
    for (int i = 0; i < 4; i++) {
        st[i] = f32_sample((lbr_signal_id_t)(LBR_SIG_CORNER_FL_BATTERY + i), 80.0f);
    }
    st[4] = (lbr_sample_t){.signal = LBR_SIG_CORNER_LINK, .type = LBR_VAL_U32, .valid = true};
    st[5] = (lbr_sample_t){.signal = LBR_SIG_CORNER_SYNCED, .type = LBR_VAL_U32, .valid = true};
    st[4].v.u32 = 0x0F;
    st[5].v.u32 = 0x02; /* FR only */
    lbr_can_encode_data(lbr_frame_def(0x30A), st, 1, &f);
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 1000000, out, 8), 6u);
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_FR)); /* IMU itself not heard */
    lbr_can_encode_heartbeat(LBR_NODE_IMU, LBR_NODE_RUN, 0, &hb);
    lbr_bus_decode(&d, &hb, 1000000, out, 8);
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_FR));
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_FL));

    /* Events decode to one sample; foreign frames to none. */
    lbr_sample_t e = {.signal    = LBR_SIG_EVENT,
                      .source    = LBR_NODE_IMU,
                      .type      = LBR_VAL_STATUS,
                      .valid     = true,
                      .t_mono_us = 7};
    lbr_can_encode_event(&e, &f);
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 10, out, 8), 1u);
    f.id = 0x7F0;
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 10, out, 8), 0u);

    /* An id above 11 bits is not the frame its low bits name. */
    lbr_can_encode_data(lbr_frame_def(0x200), v, 10, &f);
    f.id |= 0x10000u;
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 10, out, 8), 0u);
    CHECK_EQ_U(lbr_can_decode_data(&f, 10, true, out, 8), 0u);
}

static void test_bus_decoder_forgets_sync(void)
{
    lbr_bus_decoder_t d;
    lbr_bus_decoder_init(&d);
    lbr_sample_t    out[8];
    lbr_can_frame_t f;
    lbr_sample_t    e = {.signal    = LBR_SIG_EVENT,
                         .source    = LBR_NODE_POWERTRAIN,
                         .type      = LBR_VAL_STATUS,
                         .valid     = true,
                         .t_mono_us = 7};

    /* A reboot: synced again only once its next heartbeat says so. */
    lbr_can_encode_heartbeat(LBR_NODE_POWERTRAIN, LBR_NODE_RUN, 0, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_POWERTRAIN));
    e.v.u32 = lbr_event_value(LBR_EVT_BOOT, 0);
    lbr_can_encode_event(&e, &f);
    CHECK_EQ_U(lbr_bus_decode(&d, &f, 10, out, 8), 1u);
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_POWERTRAIN));
    lbr_can_encode_heartbeat(LBR_NODE_POWERTRAIN, LBR_NODE_RUN, 0, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_POWERTRAIN));

    /* Lost (and found) nodes, CAN or corner, named by the event's arg. */
    e.source = LBR_NODE_CENTRAL;
    e.v.u32  = lbr_event_value(LBR_EVT_NODE_FOUND, LBR_NODE_POWERTRAIN);
    lbr_can_encode_event(&e, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_POWERTRAIN));

    lbr_sample_t st[6];
    for (int i = 0; i < 4; i++) {
        st[i] = f32_sample((lbr_signal_id_t)(LBR_SIG_CORNER_FL_BATTERY + i), 80.0f);
    }
    st[4] = (lbr_sample_t){.signal = LBR_SIG_CORNER_LINK, .type = LBR_VAL_U32, .valid = true};
    st[5] = (lbr_sample_t){.signal = LBR_SIG_CORNER_SYNCED, .type = LBR_VAL_U32, .valid = true};
    st[4].v.u32 = 0x0F;
    st[5].v.u32 = 0x0F;
    lbr_can_encode_data(lbr_frame_def(0x30A), st, 1, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    lbr_can_encode_heartbeat(LBR_NODE_IMU, LBR_NODE_RUN, 0, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_BL));
    e.source = LBR_NODE_IMU;
    e.v.u32  = lbr_event_value(LBR_EVT_NODE_LOST, LBR_NODE_CORNER_BL);
    lbr_can_encode_event(&e, &f);
    lbr_bus_decode(&d, &f, 10, out, 8);
    CHECK(!lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_BL));
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_CORNER_FL)); /* the others keep theirs */
    CHECK(lbr_bus_decoder_synced(&d, LBR_NODE_IMU));
}

static void test_frame_record(void)
{
    lbr_can_frame_t f = {.id = 0x30A, .dlc = 3, .data = {1, 2, 3, 9, 9, 9, 9, 9}};
    uint8_t         buf[LBR_CAN_FRAME_REC_SIZE];
    CHECK_EQ_U(lbr_can_frame_rec_encode(777, LBR_CAN_REC_FLAG_OWN_TX, &f, buf, sizeof buf),
               LBR_CAN_FRAME_REC_SIZE);
    CHECK_EQ_U(buf[14 + 3], 0); /* padding zeroed: deterministic logs */
    uint64_t        t;
    uint8_t         flags;
    lbr_can_frame_t o;
    CHECK_EQ_U(lbr_can_frame_rec_decode(buf, sizeof buf, &t, &flags, &o), LBR_CAN_FRAME_REC_SIZE);
    CHECK_EQ_U(t, 777u);
    CHECK_EQ_U(flags, LBR_CAN_REC_FLAG_OWN_TX);
    CHECK_EQ_U(o.id, f.id);
    CHECK_EQ_U(o.dlc, 3);
    CHECK(memcmp(o.data, f.data, 3) == 0);
    buf[13] = 9;
    CHECK_EQ_U(lbr_can_frame_rec_decode(buf, sizeof buf, &t, &flags, &o), 0u);
}

int main(void)
{
    RUN(test_sender_map);
    RUN(test_field_codec);
    RUN(test_grid_index);
    RUN(test_data_frame_roundtrip);
    RUN(test_no_counter_frame);
    RUN(test_event_frame);
    RUN(test_unwrap);
    RUN(test_time_sync_frames);
    RUN(test_command_frame);
    RUN(test_heartbeat_and_health);
    RUN(test_bus_decoder);
    RUN(test_bus_decoder_forgets_sync);
    RUN(test_frame_record);
    TEST_MAIN_END();
}
