#include "lbr/sensor.h"
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

int main(void)
{
    RUN(test_unconfigured_reports_not_fitted);
    RUN(test_sim_is_marked_and_plausible);
    TEST_MAIN_END();
}
