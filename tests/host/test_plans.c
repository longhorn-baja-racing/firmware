#include "dashboard_plan.h"
#include "driver_inputs_plan.h"
#include "fuel_gauge_plan.h"
#include "imu_gps_plan.h"
#include "lbr/can.h"
#include "lbr/nodes.h"
#include "stress_plan.h"
#include "suspension_plan.h"
#include "temperature_plan.h"
#include "lbr_test.h"

/* Defaults the firmware ships with (CONFIG_LBR_CAN_BITRATE,
 * CONFIG_LBR_CAN_NODE_BUDGET_PERMILLE). Keep in sync with components/can/Kconfig. */
#define BITRATE          1000000u
#define NODE_BUDGET_PERM 150u

static const lbr_node_plan_t *const k_plans[] = {
    &lbr_plan_imu_gps,    &lbr_plan_suspension, &lbr_plan_driver_inputs, &lbr_plan_temperature,
    &lbr_plan_fuel_gauge, &lbr_plan_stress,     &lbr_plan_dashboard,
};
#define N_PLANS (sizeof k_plans / sizeof k_plans[0])

static lbr_sensor_t s_sensors[LBR_NODE_MAX_SENSORS];
static lbr_acq_t    s_acq[LBR_NODE_MAX_SENSORS];

static uint32_t build_and_load(const lbr_node_plan_t *p, lbr_node_config_t *cfg)
{
    lbr_node_build(p, false, s_sensors, s_acq, cfg);
    if (p->listen_all) {
        cfg->on_frame = (lbr_node_frame_cb)1; /* app installs the real handler */
    }
    return lbr_node_can_load_bps(cfg);
}

static void test_each_plan_valid(void)
{
    for (size_t i = 0; i < N_PLANS; i++) {
        lbr_node_config_t cfg;
        char              err[160] = "";
        CHECK(k_plans[i]->n_slots <= LBR_NODE_MAX_SENSORS);
        uint32_t load = build_and_load(k_plans[i], &cfg);
        int      rc   = lbr_node_config_validate(&cfg, BITRATE, NODE_BUDGET_PERM, err, sizeof err);
        if (rc != 0) {
            fprintf(stderr, "  %s: %s\n", lbr_node_name(k_plans[i]->node_id), err);
        }
        CHECK_EQ_I(rc, 0);
        printf("  %-14s %7lu bit/s (%4.1f%% of bus)\n", lbr_node_name(k_plans[i]->node_id),
               (unsigned long)load, 100.0 * load / BITRATE);
    }
}

static void test_node_ids_unique(void)
{
    for (size_t i = 0; i < N_PLANS; i++) {
        CHECK(strcmp(lbr_node_name(k_plans[i]->node_id), "?") != 0);
        for (size_t j = 0; j < i; j++) {
            CHECK(k_plans[i]->node_id != k_plans[j]->node_id);
        }
    }
}

static void test_each_signal_has_one_owner(void)
{
    for (size_t a = 0; a < N_PLANS; a++) {
        for (size_t sa = 0; sa < k_plans[a]->n_slots; sa++) {
            const lbr_slot_t *x = &k_plans[a]->slots[sa];
            for (size_t ca = 0; ca < x->n_signals; ca++) {
                for (size_t b = 0; b < a; b++) {
                    for (size_t sb = 0; sb < k_plans[b]->n_slots; sb++) {
                        const lbr_slot_t *y = &k_plans[b]->slots[sb];
                        for (size_t cb = 0; cb < y->n_signals; cb++) {
                            if (x->signals[ca] == y->signals[cb]) {
                                fprintf(stderr, "  %s owned by %s and %s\n",
                                        lbr_signal_info(x->signals[ca])->name,
                                        lbr_node_name(k_plans[a]->node_id),
                                        lbr_node_name(k_plans[b]->node_id));
                                CHECK(0);
                            }
                        }
                    }
                }
            }
        }
    }
}

static void test_system_budget(void)
{
    uint64_t total = 0;
    for (size_t i = 0; i < N_PLANS; i++) {
        lbr_node_config_t cfg;
        total += build_and_load(k_plans[i], &cfg);
    }
    /* Gateway: time sync pairs + its own heartbeat at 1 Hz. */
    uint64_t gw_frames = 2u * LBR_TIME_SYNC_HZ + LBR_HEALTH_PAGE_COUNT;
    total += gw_frames * lbr_can_frame_bits_worst(8);

    uint64_t budget = (uint64_t)BITRATE * LBR_CAN_SYSTEM_BUDGET_PERMILLE / 1000u;
    printf("  system         %7lu bit/s (%4.1f%% of bus, ceiling %u%%)\n", (unsigned long)total,
           100.0 * (double)total / BITRATE, LBR_CAN_SYSTEM_BUDGET_PERMILLE / 10u);
    CHECK(total <= budget);
}

static void test_decimation_only_with_local_log(void)
{
    for (size_t i = 0; i < N_PLANS; i++) {
        for (size_t s = 0; s < k_plans[i]->n_slots; s++) {
            if (k_plans[i]->slots[s].publish_div > 1) {
                CHECK(k_plans[i]->local_log);
            }
        }
    }
    /* Local SD logging is targeted redundancy, not every node. */
    size_t local = 0;
    for (size_t i = 0; i < N_PLANS; i++) {
        local += k_plans[i]->local_log ? 1u : 0u;
    }
    CHECK(local <= 2);
}

int main(void)
{
    RUN(test_each_plan_valid);
    RUN(test_node_ids_unique);
    RUN(test_each_signal_has_one_owner);
    RUN(test_system_budget);
    RUN(test_decimation_only_with_local_log);
    TEST_MAIN_END();
}
