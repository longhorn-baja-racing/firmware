#include <math.h>

#include "lbr/nmea.h"
#include "lbr/node.h"
#include "lbr/nodes.h"
#include "lbr/sensor.h"
#include "lbr/sensor_io.h"
#include "lbr_test.h"

static const lbr_signal_id_t k_signals[] = {
    LBR_SIG_TEMP_CVT,
    LBR_SIG_GPS_LATITUDE,
    LBR_SIG_BRAKE_SWITCH,
    LBR_SIG_GPS_SATELLITES,
};
#define N (sizeof k_signals / sizeof k_signals[0])

static void test_unconfigured_reports_not_fitted(void)
{
    lbr_sensor_t s;
    lbr_sensor_unconfigured(&s, "cvt_tc", k_signals, N);
    CHECK_EQ_I(s.init(&s), 0);
    lbr_reading_t r[N];
    memset(r, 0x55, sizeof r);
    CHECK_EQ_I(s.read(&s, 123, r), 0);
    for (size_t i = 0; i < N; i++) {
        CHECK(!r[i].valid);
        CHECK_EQ_U(r[i].reason, LBR_UNAVAIL_NOT_FITTED);
    }
    CHECK(!s.simulated);
}

static void test_sim_is_marked_and_plausible(void)
{
    lbr_sensor_t s;
    lbr_sensor_sim(&s, "sim", k_signals, N);
    CHECK(s.simulated);
    CHECK_EQ_I(s.init(&s), 0);
    lbr_reading_t r[N];
    for (uint64_t t = 0; t < 4000000; t += 50000) {
        CHECK_EQ_I(s.read(&s, t, r), 0);
        for (size_t i = 0; i < N; i++) {
            CHECK(r[i].valid);
        }
        CHECK(r[0].v.f32 >= 60.0f && r[0].v.f32 <= 100.0f); /* degC band */
    }
}

/* ---- part-independent inputs: spec parsing ------------------------------ */

static const lbr_signal_id_t k_pedals[] = {
    LBR_SIG_THROTTLE_POS,
    LBR_SIG_BRAKE_PRESS_FRONT,
    LBR_SIG_BRAKE_SWITCH,
    LBR_SIG_ENGINE_RPM,
};
#define NP (sizeof k_pedals / sizeof k_pedals[0])

static int parse_pedals(const char *spec, lbr_io_chan_t *c, char *err, size_t err_len)
{
    return lbr_io_parse(spec, k_pedals, NP, c, err, err_len);
}

static void test_io_parse_valid(void)
{
    lbr_io_chan_t c[NP];
    char          err[96] = "";
    int rc = parse_pedals("adc gpio=34 mv0=310 v0=0 mv1=2980 v1=100 lo=150 hi=3150 ; "
                          "adc gpio=35 avg=16 mv0=333 v0=0 mv1=3000 v1=13790 min=-50 max=14000;"
                          "din gpio=4 active=high pull=down debounce_ms=20;"
                          "pulse gpio=27 scale=60 stall_ms=500 edge=fall pull=up",
                          c, err, sizeof err);
    CHECK_EQ_I(rc, 0);
    if (rc != 0) {
        fprintf(stderr, "  err: %s\n", err);
        return;
    }
    CHECK_EQ_U(c[0].kind, LBR_IO_ADC);
    CHECK_EQ_I(c[0].u.adc.gpio, 34);
    CHECK_EQ_U(c[0].u.adc.avg, 4);
    CHECK(c[0].u.adc.lo_mv == 150.0f && c[0].u.adc.hi_mv == 3150.0f);
    CHECK(isinf(c[0].u.adc.min) && isinf(c[0].u.adc.max));
    CHECK_EQ_U(c[1].u.adc.avg, 16);
    CHECK(c[1].u.adc.min == -50.0f);
    CHECK_EQ_U(c[2].kind, LBR_IO_DIN);
    CHECK_EQ_U(c[2].u.din.n_gpio, 1);
    CHECK(c[2].u.din.active_high);
    CHECK_EQ_U(c[2].u.din.pull, LBR_IO_PULL_DOWN);
    CHECK_EQ_U(c[2].u.din.debounce_ms, 20);
    CHECK_EQ_U(c[3].kind, LBR_IO_PULSE);
    CHECK(c[3].u.pulse.falling);
    CHECK_EQ_U(c[3].u.pulse.stall_ms, 500);
    CHECK(c[3].u.pulse.scale == 60.0f);

    /* Not-fitted channels and a NTC curve. */
    static const lbr_signal_id_t temps[] = {LBR_SIG_TEMP_CVT, LBR_SIG_TEMP_ENGINE};
    lbr_io_chan_t                t[2];
    CHECK_EQ_I(lbr_io_parse("- ; adc gpio=36 curve=ntc r25=10000 beta=3950 rpull=10000 vsup=3300",
                            temps, 2, t, err, sizeof err),
               0);
    CHECK_EQ_U(t[0].kind, LBR_IO_NONE);
    CHECK(t[1].u.adc.ntc);

    /* Button bitmask on a u32 signal. */
    static const lbr_signal_id_t btn[] = {LBR_SIG_DASH_BUTTONS};
    lbr_io_chan_t                b;
    CHECK_EQ_I(lbr_io_parse("din gpio=4,5,18,19", btn, 1, &b, err, sizeof err), 0);
    CHECK_EQ_U(b.u.din.n_gpio, 4);
    CHECK_EQ_I(b.u.din.gpio[2], 18);
}

static void test_io_parse_rejects(void)
{
    static const char *bad[] = {
        "",
        "-;-;-",                                              /* too few channels */
        "-;-;-;-;-",                                          /* too many */
        "adc gpio=25 mv0=0 v0=0 mv1=1 v1=1;-;-;-",            /* ADC2 pin */
        "adc gpio=34 mv0=0 v0=0 mv1=1;-;-;-",                 /* missing v1 */
        "adc gpio=34 mv0=5 v0=0 mv1=5 v1=1;-;-;-",            /* degenerate */
        "adc gpio=34 mv0=0 v0=0 mv1=1 v1=1 r25=1;-;-;-",      /* lin + ntc keys */
        "adc gpio=34 mv0=0 v0=0 mv1=1 v1=1 lo=10 hi=5;-;-;-", /* empty window */
        "adc gpio=34 mv0=0 v0=0 mv1=1 v1=1 mv0=2;-;-;-",      /* duplicate key */
        "adc gpio=34 mv0=0 v0=x mv1=1 v1=1;-;-;-",            /* bad number */
        "adc gpio=34 mv0=0 v0=nan mv1=1 v1=1;-;-;-",          /* not finite */
        "adc gpio=34 mv0=0 v0=0 mv1=1 v1=1 scale=2;-;-;-",    /* key of another kind */
        "adc mv0=0 v0=0 mv1=1 v1=1;-;-;-",                    /* no gpio */
        "-;-;adc gpio=34 mv0=0 v0=0 mv1=1 v1=1;-",            /* adc on a bool */
        "-;-;din gpio=4,5;-",                                 /* bool with two pins */
        "-;-;din gpio=35;-",                                  /* pull-up on input-only */
        "-;-;din gpio=7 pull=none;-",                         /* flash pin */
        "-;-;-;pulse gpio=27",                                /* no scale */
        "-;-;-;pulse gpio=27 scale=-1",                       /* negative scale */
        "-;-;din gpio=27;pulse gpio=27 scale=1",              /* pin used twice */
        "-;-;-;servo gpio=27",                                /* unknown kind */
        "-;;-;-",                                             /* empty channel */
        "-;-;din gpio=4 debounce_ms=5000;-",                  /* out of range */
        "-;-;din gpio=4 active=sideways;-",                   /* bad enum */
        "-;-;din gpio=4 =1;-",                                /* empty key */
        "-;-;din gpio=4\x01;-",                               /* not ASCII text */
    };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        lbr_io_chan_t c[NP];
        char          err[96] = "";
        if (parse_pedals(bad[i], c, err, sizeof err) == 0) {
            fprintf(stderr, "  accepted bad spec %u: %s\n", (unsigned)i, bad[i]);
            CHECK(false);
        } else {
            CHECK(err[0] != '\0');
        }
    }
    lbr_io_chan_t c[NP];
    CHECK(lbr_io_parse(NULL, k_pedals, NP, c, NULL, 0) < 0);
}

/* ---- conversions ---------------------------------------------------------- */

static void test_io_adc_convert(void)
{
    lbr_io_chan_t c;
    char          err[96];
    CHECK_EQ_I(lbr_io_parse("adc gpio=34 mv0=500 v0=0 mv1=2500 v1=100 lo=200 hi=2800 min=-5 "
                            "max=105",
                            k_pedals, 1, &c, err, sizeof err),
               0);
    lbr_reading_t r;
    lbr_io_adc_convert(&c, 1500.0f, &r);
    CHECK(r.valid && fabsf(r.v.f32 - 50.0f) < 1e-4f);
    lbr_io_adc_convert(&c, 500.0f, &r);
    CHECK(r.valid && fabsf(r.v.f32) < 1e-4f);
    lbr_io_adc_convert(&c, 100.0f, &r); /* open circuit: below the window */
    CHECK(!r.valid);
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_SENSOR_FAULT);
    lbr_io_adc_convert(&c, 3000.0f, &r); /* short to supply */
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_SENSOR_FAULT);
    lbr_io_adc_convert(&c, 2700.0f, &r); /* 110 %: inside the window, beyond max */
    CHECK(!r.valid);
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_OUT_OF_RANGE);
    lbr_io_adc_convert(&c, NAN, &r);
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_SENSOR_FAULT);

    static const lbr_signal_id_t t[] = {LBR_SIG_TEMP_CVT};
    CHECK_EQ_I(lbr_io_parse("adc gpio=39 curve=ntc r25=10000 beta=3950 rpull=10000 vsup=3300", t, 1,
                            &c, err, sizeof err),
               0);
    lbr_io_adc_convert(&c, 1650.0f, &r); /* R = r25 */
    CHECK(r.valid && fabsf(r.v.f32 - 25.0f) < 0.01f);
    lbr_io_adc_convert(&c, 600.0f, &r); /* lower R = hotter */
    CHECK(r.valid && r.v.f32 > 60.0f && r.v.f32 < 70.0f);
    lbr_io_adc_convert(&c, 0.0f, &r); /* shorted thermistor */
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_SENSOR_FAULT);
    lbr_io_adc_convert(&c, 3300.0f, &r); /* open thermistor */
    CHECK_EQ_U(r.reason, LBR_UNAVAIL_SENSOR_FAULT);
}

static void test_io_pulse_rate(void)
{
    lbr_io_pulse_t st    = {0};
    uint32_t       count = 0;
    uint64_t       edge  = 0;

    /* Nothing seen yet. */
    CHECK(lbr_io_pulse_update(&st, count, edge, 1000, 2000) == 0.0f);

    /* 100 Hz: an edge every 10 ms, read every 7 ms (not aligned). */
    float hz = 0.0f;
    for (uint64_t now = 0; now <= 1000000; now += 7000) {
        while (edge + 10000 <= now) {
            edge += 10000;
            count++;
        }
        hz = lbr_io_pulse_update(&st, count, edge, now, 2000);
    }
    CHECK(fabsf(hz - 100.0f) < 0.01f);

    /* Edges stop: decays as 1/elapsed, then reads zero after stall_ms. */
    uint64_t last = edge;
    hz            = lbr_io_pulse_update(&st, count, edge, last + 50000, 2000);
    CHECK(fabsf(hz - 20.0f) < 0.01f);
    hz = lbr_io_pulse_update(&st, count, edge, last + 2000000, 2000);
    CHECK(hz == 0.0f);
    CHECK(!st.have_ref);

    /* Restart: the first edge only re-anchors; the second gives the rate. */
    count++;
    edge = last + 3000000;
    CHECK(lbr_io_pulse_update(&st, count, edge, edge + 1, 2000) == 0.0f);
    count += 2;
    edge += 40000; /* two edges in 40 ms = 50 Hz */
    hz = lbr_io_pulse_update(&st, count, edge, edge + 1, 2000);
    CHECK(fabsf(hz - 50.0f) < 0.01f);

    /* Counter wrap. */
    lbr_io_pulse_t w = {.ref_count = 0xFFFFFFF0u, .ref_edge_us = 1000, .have_ref = true};
    hz               = lbr_io_pulse_update(&w, 0x00000010u, 1000 + 32000, 1000 + 32000, 2000);
    CHECK(fabsf(hz - 1000.0f) < 0.01f);
}

static void test_io_din_debounce(void)
{
    lbr_io_din_t st = {0};
    CHECK_EQ_U(lbr_io_din_update(&st, 0, 0, 10), 0);
    CHECK_EQ_U(lbr_io_din_update(&st, 1, 1000, 10), 0);  /* bounce starts */
    CHECK_EQ_U(lbr_io_din_update(&st, 0, 3000, 10), 0);  /* bounces back */
    CHECK_EQ_U(lbr_io_din_update(&st, 1, 4000, 10), 0);  /* pending again */
    CHECK_EQ_U(lbr_io_din_update(&st, 1, 13000, 10), 0); /* 9 ms stable */
    CHECK_EQ_U(lbr_io_din_update(&st, 1, 14000, 10), 1); /* 10 ms stable */
    lbr_io_din_t z = {0};
    CHECK_EQ_U(lbr_io_din_update(&z, 5, 0, 0), 5);
    CHECK_EQ_U(lbr_io_din_update(&z, 6, 1, 0), 6); /* debounce 0 follows at once */
}

static void test_config_text_records_params(void)
{
    static const char spec[] = "adc gpio=36 mv0=100 v0=0 mv1=3000 v1=100";
    lbr_sensor_t      s;
    lbr_sensor_unconfigured(&s, "fuel", k_pedals, 1);
    s.params              = spec;
    lbr_acq_t         acq = {.sensor = &s, .rate_hz = 5, .publish_div = 1};
    lbr_node_config_t cfg = {
        .node_id = LBR_NODE_FUEL_GAUGE, .acq = &acq, .n_acq = 1, .health_period_ms = 1000};
    char text[LBR_NODE_CONFIG_TEXT_MAX];
    lbr_node_config_text(&cfg, text, sizeof text);
    CHECK(strstr(text, "sensor.0.params=adc gpio=36 mv0=100 v0=0 mv1=3000 v1=100\n") != NULL);
    uint32_t with = lbr_node_config_crc(&cfg);
    s.params      = NULL;
    CHECK(lbr_node_config_crc(&cfg) != with); /* calibration change = config change */
}

/* ---- NMEA GPS ---------------------------------------------------------------- */

static const lbr_signal_id_t k_gps[] = {
    LBR_SIG_GPS_LATITUDE, LBR_SIG_GPS_LONGITUDE, LBR_SIG_GPS_ALTITUDE,   LBR_SIG_GPS_SPEED,
    LBR_SIG_GPS_COURSE,   LBR_SIG_GPS_FIX,       LBR_SIG_GPS_SATELLITES, LBR_SIG_GPS_HDOP,
    LBR_SIG_GPS_UTC,      LBR_SIG_TEMP_CVT,
};
#define NG (sizeof k_gps / sizeof k_gps[0])

static const char k_fix_burst[] =
    "$GNGGA,123456.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*7D\r\n"
    "$GNRMC,123456.00,A,4807.038,N,01131.000,E,022.4,084.4,181026,,,A*45\r\n"
    "$GNGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*27\r\n";

static void feed_str(lbr_nmea_t *p, const char *s, uint64_t now)
{
    lbr_nmea_feed(p, (const uint8_t *)s, strlen(s), now);
}

static void test_nmea_fix(void)
{
    lbr_nmea_t p;
    lbr_nmea_init(&p);
    lbr_reading_t r[NG];

    /* Before any data: all NO_DATA, non-GPS signal NOT_FITTED. */
    lbr_nmea_fill(&p, 1000, k_gps, NG, r);
    for (size_t i = 0; i < NG - 1; i++) {
        CHECK(!r[i].valid);
        CHECK_EQ_U(r[i].reason, LBR_UNAVAIL_NO_DATA);
    }
    CHECK_EQ_U(r[NG - 1].reason, LBR_UNAVAIL_NOT_FITTED);

    /* Fed in odd-sized pieces, as a UART delivers it. */
    const size_t n    = strlen(k_fix_burst);
    uint32_t     good = 0;
    for (size_t off = 0; off < n; off += 7) {
        size_t len = n - off < 7 ? n - off : 7;
        good += lbr_nmea_feed(&p, (const uint8_t *)k_fix_burst + off, len, 5000000);
    }
    CHECK_EQ_U(good, 3);
    CHECK_EQ_U(p.checksum_errors, 0);

    lbr_nmea_fill(&p, 5100000, k_gps, NG, r);
    CHECK(r[0].valid && r[0].v.i32 == 481173000);
    CHECK(r[1].valid && r[1].v.i32 == 115166667);
    CHECK(r[2].valid && fabsf(r[2].v.f32 - 545.4f) < 1e-3f);
    CHECK(r[3].valid && fabsf(r[3].v.f32 - 22.4f * 0.514444f) < 1e-3f);
    CHECK(r[4].valid && fabsf(r[4].v.f32 - 84.4f) < 1e-3f);
    CHECK(r[5].valid && r[5].v.u32 == 3);
    CHECK(r[6].valid && r[6].v.u32 == 8);
    CHECK(r[7].valid && fabsf(r[7].v.f32 - 0.9f) < 1e-4f);
    CHECK(r[8].valid && r[8].v.u32 == 1792326896u);

    /* Stream goes quiet: everything becomes NO_DATA, never a stale value. */
    lbr_nmea_fill(&p, 5000000 + LBR_NMEA_STALE_US, k_gps, NG, r);
    for (size_t i = 0; i < NG - 1; i++) {
        CHECK(!r[i].valid);
    }

    /* Fix lost: fix type 0 is a valid reading, position is NO_DATA. */
    feed_str(&p,
             "$GPGGA,123457.00,,,,,0,00,,,M,,M,,*4E\r\n"
             "$GPRMC,123457.00,V,,,,,,,181026,,,N*77\r\n",
             9000000);
    lbr_nmea_fill(&p, 9000001, k_gps, NG, r);
    CHECK(r[5].valid && r[5].v.u32 == 0);
    CHECK(!r[0].valid && r[0].reason == LBR_UNAVAIL_NO_DATA);
    CHECK(!r[8].valid); /* receiver clock without a fix is not trusted */
    CHECK(r[6].valid && r[6].v.u32 == 0);
    CHECK(!r[7].valid); /* empty HDOP field */
}

static void test_nmea_rejects_garbage(void)
{
    lbr_nmea_t p;
    lbr_nmea_init(&p);
    /* Bad checksum, missing checksum, overlong line, binary noise, then a good
     * sentence: only the last one counts. */
    feed_str(&p, "$GNGGA,123456.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*7E\r\n", 1);
    feed_str(&p, "$GNGGA,123456.00,4807.038,N\r\n", 1);
    char longline[200];
    memset(longline, 'A', sizeof longline);
    longline[0]                   = '$';
    longline[sizeof longline - 1] = '\n';
    lbr_nmea_feed(&p, (const uint8_t *)longline, sizeof longline, 1);
    const uint8_t noise[] = {0xB5, 0x62, 0x01, 0x07, 0x00, '$', 0xFF, 0x00, '\n'};
    lbr_nmea_feed(&p, noise, sizeof noise, 1);
    CHECK_EQ_U(p.sentences, 0);
    CHECK_EQ_U(p.checksum_errors, 2);
    CHECK(p.overlong >= 2);
    feed_str(&p, k_fix_burst, 2);
    CHECK_EQ_U(p.sentences, 3);

    /* Out-of-range coordinates are not a position. */
    lbr_nmea_t q;
    lbr_nmea_init(&q);
    feed_str(&q, "$GNGGA,1,9107.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*61\r\n", 1);
    CHECK_EQ_U(q.sentences, 1);
    CHECK_EQ_U(q.pos_us, 0);
}

static void test_nmea_unix_time(void)
{
    CHECK_EQ_U(lbr_nmea_unix_time(1970, 1, 1, 0, 0, 0), 0);
    CHECK_EQ_U(lbr_nmea_unix_time(2000, 2, 29, 0, 0, 0), 951782400u);
    CHECK_EQ_U(lbr_nmea_unix_time(2026, 10, 18, 12, 34, 56), 1792326896u);
    CHECK_EQ_U(lbr_nmea_unix_time(2026, 13, 1, 0, 0, 0), 0);
}

int main(void)
{
    RUN(test_unconfigured_reports_not_fitted);
    RUN(test_sim_is_marked_and_plausible);
    RUN(test_io_parse_valid);
    RUN(test_io_parse_rejects);
    RUN(test_io_adc_convert);
    RUN(test_io_pulse_rate);
    RUN(test_io_din_debounce);
    RUN(test_config_text_records_params);
    RUN(test_nmea_fix);
    RUN(test_nmea_rejects_garbage);
    RUN(test_nmea_unix_time);
    TEST_MAIN_END();
}
