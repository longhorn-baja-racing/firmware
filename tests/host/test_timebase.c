#include <stdlib.h>

#include "lbr/timebase.h"
#include "lbr_test.h"
#include "port_host.h"

/* Simple deterministic jitter source. */
static uint32_t s_rng = 12345;
static int32_t  jitter(int32_t amp)
{
    s_rng = s_rng * 1103515245u + 12345u;
    return (int32_t)((s_rng >> 8) % (uint32_t)(2 * amp + 1)) - amp;
}

/* Local clock runs `ppm` fast relative to the gateway, from `offset`. */
static uint64_t local_at(uint64_t gw_us, int32_t ppm, uint64_t offset)
{
    return offset + gw_us + (uint64_t)((int64_t)gw_us * ppm / 1000000);
}

static void test_first_sync_steps(void)
{
    lbr_time_servo_t s;
    lbr_time_servo_init(&s);
    CHECK(!s.synced);
    CHECK_EQ_U(lbr_time_servo_map(&s, 500), 500u); /* unsynced: local passthrough */
    CHECK_EQ_I(lbr_time_servo_update(&s, 10000000, 2000, 7), LBR_TIME_UPD_FIRST);
    CHECK(s.synced);
    CHECK_EQ_U(lbr_time_servo_map(&s, 3000), 10001000u);
}

static void test_converges_with_drift_and_jitter(void)
{
    const int32_t    ppm    = 120; /* worse than a typical crystal */
    const uint64_t   offset = 987654321;
    lbr_time_servo_t s;
    lbr_time_servo_init(&s);

    int32_t worst_late = 0;
    for (int i = 0; i < 600; i++) { /* 10 minutes at 1 Hz */
        uint64_t          gw    = 1000000ull + (uint64_t)i * 1000000ull;
        uint64_t          local = local_at(gw, ppm, offset) + (uint64_t)(int64_t)jitter(15);
        lbr_time_update_t r     = lbr_time_servo_update(&s, gw, local, 1);
        if (i > 0) {
            CHECK(r == LBR_TIME_UPD_SLEW);
        }
        if (i >= 60) {
            int32_t e = s.last_error_us < 0 ? -s.last_error_us : s.last_error_us;
            if (e > worst_late) {
                worst_late = e;
            }
        }
    }
    /* After a minute, residual error stays within jitter-scale bounds. */
    CHECK(worst_late < 40);
    /* Rate estimate converged near the true drift (local fast => shared slower). */
    CHECK(abs(s.rate_ppb + ppm * 1000) < 10000);
    CHECK_EQ_U(s.step_count, 0u);

    /* Between syncs, mapped time tracks the gateway closely. */
    uint64_t gw    = 601500000ull;
    uint64_t local = local_at(gw, ppm, offset);
    int64_t  err   = (int64_t)lbr_time_servo_map(&s, local) - (int64_t)gw;
    CHECK(err > -60 && err < 60);
}

static void test_step_and_monotonic(void)
{
    lbr_time_servo_t s;
    lbr_time_servo_init(&s);
    lbr_time_servo_update(&s, 1000000, 1000000, 1);
    uint64_t t1 = lbr_time_servo_now(&s, 1999000);
    CHECK_EQ_U(t1, 1999000u);

    /* Gateway says we are 10 ms ahead: step backwards. */
    CHECK_EQ_I(lbr_time_servo_update(&s, 1990000, 2000000, 1), LBR_TIME_UPD_STEP);
    CHECK_EQ_U(s.step_count, 1u);
    /* Output never goes backwards: it holds, then resumes. */
    uint64_t prev = lbr_time_servo_now(&s, 2000000);
    CHECK_EQ_U(prev, 1999000u);
    for (uint64_t l = 2000000; l < 2030000; l += 1000) {
        uint64_t t = lbr_time_servo_now(&s, l);
        CHECK(t >= prev);
        prev = t;
    }
    CHECK_EQ_U(lbr_time_servo_now(&s, 2030000), 2020000u);
}

static void test_epoch_change(void)
{
    lbr_time_servo_t s;
    lbr_time_servo_init(&s);
    lbr_time_servo_update(&s, 50000000, 1000, 1);
    CHECK(lbr_time_servo_now(&s, 2000) == 50001000u);
    /* Gateway rebooted: new epoch, time restarts near zero. */
    CHECK_EQ_I(lbr_time_servo_update(&s, 3000, 3000, 2), LBR_TIME_UPD_EPOCH_CHANGE);
    CHECK_EQ_U(s.epoch_changes, 1u);
    CHECK_EQ_U(lbr_time_servo_now(&s, 4000), 4000u); /* guard reset for new timeline */
}

static void test_ignores_stale_and_master(void)
{
    lbr_time_servo_t s;
    lbr_time_servo_init(&s);
    lbr_time_servo_update(&s, 1000, 1000, 1);
    CHECK_EQ_I(lbr_time_servo_update(&s, 900, 900, 1), LBR_TIME_UPD_IGNORED);
    lbr_time_servo_set_master(&s);
    CHECK(s.synced);
    CHECK_EQ_I(lbr_time_servo_update(&s, 5, 5, 1), LBR_TIME_UPD_IGNORED);
    CHECK_EQ_U(lbr_time_servo_map(&s, 123456), 123456u);
}

static void test_sync_pairing(void)
{
    lbr_time_sync_rx_t rx = {0};
    uint64_t           gw, local;
    uint16_t           epoch;
    CHECK(!lbr_time_sync_on_follow_up(&rx, 1, 100, &gw, &local, &epoch));
    CHECK_EQ_U(rx.unmatched, 1u);
    lbr_time_sync_on_sync(&rx, 5, 9, 4444);
    CHECK(!lbr_time_sync_on_follow_up(&rx, 4, 100, &gw, &local, &epoch)); /* wrong seq */
    CHECK(lbr_time_sync_on_follow_up(&rx, 5, 100, &gw, &local, &epoch));
    CHECK_EQ_U(gw, 100u);
    CHECK_EQ_U(local, 4444u);
    CHECK_EQ_U(epoch, 9u);
    CHECK(!lbr_time_sync_on_follow_up(&rx, 5, 100, &gw, &local, &epoch)); /* consumed */
}

static void test_global_wrapper(void)
{
    port_host_set_time(1000);
    CHECK(!lbr_time_is_synced());
    CHECK_EQ_U(lbr_time_now_us(), 1000u);
    lbr_time_sync_apply(77000, 1000, 3);
    CHECK(lbr_time_is_synced());
    port_host_set_time(2000);
    CHECK_EQ_U(lbr_time_now_us(), 78000u);
    CHECK_EQ_U(lbr_time_local_to_shared(1500), 77500u);
    lbr_time_set_master();
    CHECK_EQ_U(lbr_time_now_us(), 2000u);
}

int main(void)
{
    RUN(test_first_sync_steps);
    RUN(test_converges_with_drift_and_jitter);
    RUN(test_step_and_monotonic);
    RUN(test_epoch_change);
    RUN(test_ignores_stale_and_master);
    RUN(test_sync_pairing);
    RUN(test_global_wrapper);
    TEST_MAIN_END();
}
