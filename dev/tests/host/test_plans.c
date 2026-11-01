#include "central_plan.h"
#include "corner_plan.h"
#include "dashboard_plan.h"
#include "driver_input_plan.h"
#include "imu_plan.h"
#include "lbr/can.h"
#include "lbr/frames.h"
#include "lbr/nodes.h"
#include "lbr_test.h"
#include "powertrain_plan.h"
#include "stress_plan.h"
#include "temperature_plan.h"

/* Defaults the firmware ships with (CONFIG_LBR_CAN_BITRATE and
 * CONFIG_LBR_CAN_TX_QUEUE_LEN). Keep in sync with car/components/can/Kconfig. */
#define BITRATE      500000u
#define TX_QUEUE_LEN 64u

/* The CAN nodes (DAQ-WIRING.md "CAN node 1..10"). */
static const lbr_node_plan_t *const k_can_plans[] = {
    &lbr_plan_imu,       &lbr_plan_powertrain, &lbr_plan_driver_input, &lbr_plan_temperature,
    &lbr_plan_stress,    &lbr_plan_central,    &lbr_plan_corner[0],    &lbr_plan_corner[1],
    &lbr_plan_corner[2], &lbr_plan_corner[3],
};
#define N_CAN (sizeof k_can_plans / sizeof k_can_plans[0])

/* The nodes off the bus. */
static const lbr_node_plan_t *const k_wireless_plans[] = {
    &lbr_plan_dashboard,
};
#define N_WIRELESS (sizeof k_wireless_plans / sizeof k_wireless_plans[0])

static lbr_sensor_t s_sensors[LBR_NODE_MAX_SENSORS];
static lbr_acq_t    s_acq[LBR_NODE_MAX_SENSORS];

static void test_each_plan_valid(void)
{
    char err[160];
    for (size_t i = 0; i < N_CAN; i++) {
        err[0] = '\0';
        int rc = lbr_plan_check(k_can_plans[i], true, err, sizeof err);
        if (rc != 0) {
            fprintf(stderr, "  %s: %s\n", lbr_node_name(k_can_plans[i]->node_id), err);
        }
        CHECK_EQ_I(rc, 0);
        if (k_can_plans[i]->node_id == LBR_NODE_CENTRAL) {
            continue; /* the Central runs its slots in gw_core, not the node runtime */
        }
        lbr_node_config_t cfg;
        lbr_node_build(k_can_plans[i], lbr_sensor_unconfigured, s_sensors, s_acq, &cfg);
        rc = lbr_node_config_validate(&cfg, err, sizeof err);
        if (rc != 0) {
            fprintf(stderr, "  %s config: %s\n", lbr_node_name(cfg.node_id), err);
        }
        CHECK_EQ_I(rc, 0);
    }
    for (size_t i = 0; i < N_WIRELESS; i++) {
        err[0] = '\0';
        int rc = lbr_plan_check(k_wireless_plans[i], false, err, sizeof err);
        if (rc != 0) {
            fprintf(stderr, "  %s: %s\n", lbr_node_name(k_wireless_plans[i]->node_id), err);
        }
        CHECK_EQ_I(rc, 0);
    }
    /* A CAN plan is not a wireless plan and vice versa. */
    CHECK(lbr_plan_check(&lbr_plan_imu, false, err, sizeof err) != 0);
    CHECK(lbr_plan_check(&lbr_plan_dashboard, true, err, sizeof err) != 0);
}

static void test_node_ids_unique(void)
{
    const lbr_node_plan_t *all[N_CAN + N_WIRELESS];
    for (size_t i = 0; i < N_CAN; i++) {
        all[i] = k_can_plans[i];
    }
    for (size_t i = 0; i < N_WIRELESS; i++) {
        all[N_CAN + i] = k_wireless_plans[i];
    }
    for (size_t i = 0; i < N_CAN + N_WIRELESS; i++) {
        CHECK(strcmp(lbr_node_name(all[i]->node_id), "?") != 0);
        for (size_t j = 0; j < i; j++) {
            CHECK(all[i]->node_id != all[j]->node_id);
        }
    }
}

static bool is_system_id(uint32_t id)
{
    return id == LBR_CAN_ID_TIME_SYNC || id == LBR_CAN_ID_TIME_FOLLOW_UP ||
           id == LBR_CAN_ID_COMMAND ||
           (id > LBR_CAN_ID_EVENT_BASE && id <= LBR_CAN_ID_EVENT_BASE + LBR_NODE_CAN_MAX) ||
           (id > LBR_CAN_ID_HEARTBEAT_BASE && id <= LBR_CAN_ID_HEARTBEAT_BASE + LBR_NODE_CAN_MAX) ||
           id >= LBR_CAN_ID_HEALTH_BASE;
}

static void test_frame_table(void)
{
    size_t n = lbr_frame_count();
    CHECK(n > 0);
    for (size_t i = 0; i < n; i++) {
        const lbr_frame_def_t *d = lbr_frame_at(i);
        CHECK(d->can_id <= LBR_CAN_ID_MAX && !is_system_id(d->can_id));
        CHECK(i == 0 || lbr_frame_at(i - 1)->can_id < d->can_id); /* sorted, unique */
        CHECK(lbr_frame_def(d->can_id) == d);
        CHECK(lbr_node_is_can(d->sender));
        CHECK(d->rate_hz > 0 && 1000000u % d->rate_hz == 0);
        CHECK(d->dlc >= 1 && d->dlc <= 8);
        CHECK(d->n_fields >= 1 && d->n_fields <= LBR_FRAME_MAX_FIELDS);
        CHECK(d->name != NULL && d->name[0] != '\0');
        uint8_t used = 0;
        if (d->counter_offset != LBR_FRAME_NO_COUNTER) {
            CHECK(d->counter_offset + 2u <= d->dlc);
            used |= (uint8_t)(3u << d->counter_offset);
        }
        for (size_t f = 0; f < d->n_fields; f++) {
            const lbr_field_def_t   *fd   = &d->fields[f];
            const lbr_signal_info_t *info = lbr_signal_info(fd->signal);
            unsigned                 size = lbr_field_enc_size((lbr_field_enc_t)fd->enc);
            CHECK(info != NULL);
            CHECK(size > 0 && fd->offset + size <= d->dlc);
            uint8_t bits = (uint8_t)(((1u << size) - 1u) << fd->offset);
            CHECK((used & bits) == 0); /* fields never overlap */
            used |= bits;
            CHECK(fd->scale > 0.0f);
            if (info != NULL && info->type != LBR_VAL_F32) {
                CHECK(fd->scale == 1.0f); /* integer signals travel unscaled */
            }
        }
    }
    /* Every catalog signal travels in exactly one frame (events have their own). */
    for (size_t i = 0; i < lbr_signal_count(); i++) {
        lbr_signal_id_t sig   = lbr_signal_at(i)->id;
        size_t          count = 0;
        for (size_t k = 0; k < n; k++) {
            for (size_t f = 0; f < lbr_frame_at(k)->n_fields; f++) {
                count += lbr_frame_at(k)->fields[f].signal == sig ? 1u : 0u;
            }
        }
        if (sig == LBR_SIG_EVENT || sig == LBR_SIG_FAULT) {
            CHECK_EQ_U(count, 0u);
        } else {
            if (count != 1) {
                fprintf(stderr, "  %s is in %u frames\n", lbr_signal_at(i)->name, (unsigned)count);
            }
            CHECK_EQ_U(count, 1u);
        }
    }
}

/* Each frame is filled by its sender's plan. */
static void test_every_frame_has_a_producer(void)
{
    for (size_t i = 0; i < lbr_frame_count(); i++) {
        const lbr_frame_def_t *d     = lbr_frame_at(i);
        bool                   found = false;
        const lbr_node_plan_t *plans[N_CAN + N_WIRELESS];
        for (size_t k = 0; k < N_CAN; k++) {
            plans[k] = k_can_plans[k];
        }
        for (size_t k = 0; k < N_WIRELESS; k++) {
            plans[N_CAN + k] = k_wireless_plans[k];
        }
        for (size_t p = 0; p < N_CAN + N_WIRELESS && !found; p++) {
            if (plans[p]->node_id != d->sender) {
                continue;
            }
            for (size_t s = 0; s < plans[p]->n_slots; s++) {
                for (size_t c = 0; c < plans[p]->slots[s].n_signals; c++) {
                    found = found || plans[p]->slots[s].signals[c] == d->fields[0].signal;
                }
            }
        }
        if (!found) {
            fprintf(stderr, "  frame %s (0x%03x) has no producer\n", d->name, d->can_id);
        }
        CHECK(found);
    }
}

static void test_system_budget(void)
{
    uint64_t total = 0;
    for (size_t i = 0; i < N_CAN; i++) {
        lbr_node_config_t cfg  = {.node_id          = k_can_plans[i]->node_id,
                                  .health_period_ms = k_can_plans[i]->health_period_ms};
        uint32_t          load = lbr_node_can_load_bps(&cfg);
        printf("  %-14s %7lu bit/s (%4.1f%% of %u kbit/s)\n", lbr_node_name(cfg.node_id),
               (unsigned long)load, 100.0 * load / BITRATE, BITRATE / 1000u);
        total += load;
    }
    /* The Central also sends the time-sync pair. */
    total += 2u * LBR_TIME_SYNC_HZ * lbr_can_frame_bits_worst(8);

    uint64_t budget = (uint64_t)BITRATE * LBR_CAN_SYSTEM_BUDGET_PERMILLE / 1000u;
    printf("  system         %7lu bit/s (%4.1f%% of bus, ceiling %u%%)\n", (unsigned long)total,
           100.0 * (double)total / BITRATE, LBR_CAN_SYSTEM_BUDGET_PERMILLE / 10u);
    CHECK(total <= budget);
}

static void test_local_logs_are_targeted(void)
{
    /* Local SD logging is targeted redundancy, not every node. The Central's
     * card is the authoritative log and is not a plan flag. */
    size_t local = 0;
    for (size_t i = 0; i < N_CAN; i++) {
        local += k_can_plans[i]->local_log ? 1u : 0u;
    }
    for (size_t i = 0; i < N_WIRELESS; i++) {
        CHECK(!k_wireless_plans[i]->local_log);
    }
    CHECK(local <= 2);
}

/*
 * Every slot of a node can come due in the same instant as its heartbeat
 * (their periods divide one second), and all of those frames are queued
 * before the first one is on the wire. The TX queue must hold that burst
 * plus a couple of events, or frames are dropped every second.
 */
static void test_burst_fits_tx_queue(void)
{
    for (size_t i = 0; i < N_CAN; i++) {
        uint8_t node  = k_can_plans[i]->node_id;
        size_t  burst = 1u + LBR_HEALTH_PAGE_COUNT + 2u; /* heartbeat, pages, EVENT/FAULT */
        size_t  cap   = TX_QUEUE_LEN;
        for (size_t f = 0; f < lbr_frame_count(); f++) {
            burst += lbr_frame_at(f)->sender == node ? 1u : 0u;
        }
        if (node == LBR_NODE_CENTRAL) {
            burst += 2u + 1u; /* time sync pair, command */
        }
        if (burst > cap) {
            fprintf(stderr, "  %s: worst-case burst %u > TX queue %u\n", lbr_node_name(node),
                    (unsigned)burst, (unsigned)cap);
        }
        CHECK(burst <= cap);
    }
}

int main(void)
{
    RUN(test_each_plan_valid);
    RUN(test_node_ids_unique);
    RUN(test_frame_table);
    RUN(test_every_frame_has_a_producer);
    RUN(test_system_budget);
    RUN(test_local_logs_are_targeted);
    RUN(test_burst_fits_tx_queue);
    TEST_MAIN_END();
}
