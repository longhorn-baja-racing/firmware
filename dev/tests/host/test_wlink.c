/*
 * The cockpit dashboard link (Central <-> dashboard) through its pure cores
 * and fake platforms.
 */
#include <string.h>

#include "lbr/dash_core.h"
#include "lbr/dash_link.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr/wlink.h"
#include "lbr_test.h"

static void test_seq_window(void)
{
    lbr_seq_window_t w    = {0};
    uint32_t         gaps = 0;
    CHECK_EQ_I(lbr_seq_window_take(&w, 10, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 0u); /* 11 and 12 may still come */
    CHECK_EQ_I(lbr_seq_window_take(&w, 11, &gaps), LBR_SEQ_LATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 11, &gaps), LBR_SEQ_DUPLICATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13, &gaps), LBR_SEQ_DUPLICATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13 + 32, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 1u); /* 12 left the window unseen; 14 .. 44 still may come */
    CHECK_EQ_I(lbr_seq_window_take(&w, 12, &gaps), LBR_SEQ_OLD);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13 + 64, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 1u + 31u); /* now 14 .. 44 are gone too */
    /* Across the 32-bit wrap. */
    lbr_seq_window_t v = {0};
    gaps               = 0;
    CHECK_EQ_I(lbr_seq_window_take(&v, 0xFFFFFFFFu, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_I(lbr_seq_window_take(&v, 0, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 0u);

    /* A restart (outage, sender reboot) still counts what was known lost
     * inside the window, and nothing twice. */
    lbr_seq_window_t r = {0};
    gaps               = 0;
    lbr_seq_window_take(&r, 100, &gaps);
    lbr_seq_window_take(&r, 103, &gaps); /* 101, 102 missing, not yet out of the window */
    CHECK_EQ_U(gaps, 0u);
    lbr_seq_window_restart(&r, &gaps);
    CHECK_EQ_U(gaps, 2u);
    lbr_seq_window_restart(&r, &gaps);
    CHECK_EQ_U(gaps, 2u);
    CHECK_EQ_I(lbr_seq_window_take(&r, 5, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 2u);
}

/* ------------------------------------------------------------------------ */
/* Central <-> dashboard                                                     */

static int ok_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}

typedef struct {
    uint64_t mono;
    uint8_t  pkt[256];
    size_t   len;
} dash_fake_t;

static uint64_t df_mono(void *ctx)
{
    return ((dash_fake_t *)ctx)->mono;
}
static bool df_send(void *ctx, const uint8_t *pkt, size_t len)
{
    dash_fake_t *f = ctx;
    memcpy(f->pkt, pkt, len < sizeof f->pkt ? len : sizeof f->pkt);
    f->len = len;
    return true;
}

static const lbr_signal_id_t k_dash_batt[] = {LBR_SIG_DASH_BATTERY};

static int dash_batt_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)self;
    (void)t_us;
    out[0] = (lbr_reading_t){.valid = true, .v.f32 = 91.0f};
    return 0;
}

static void test_dashboard_round_trip(void)
{
    /* Central side: values from the bus and one node's health. */
    lbr_value_table_t values;
    lbr_node_table_t  nodes;
    lbr_dash_link_t   link;
    lbr_value_table_init(&values, 2000000);
    for (size_t i = 0; i < lbr_dash_signal_count; i++) {
        lbr_value_table_watch(&values, lbr_dash_signals[i], 0);
    }
    lbr_node_table_init(&nodes, 3000000);
    lbr_dash_link_init(&link);
    lbr_sample_t rpm = {.signal = LBR_SIG_ENGINE_RPM,
                        .source = LBR_NODE_POWERTRAIN,
                        .type   = LBR_VAL_F32,
                        .valid  = true,
                        .v.f32  = 3600.0f};
    lbr_value_table_offer(&values, &rpm, 1000);
    lbr_sample_t fuel = {.signal = LBR_SIG_FUEL_LEVEL, .type = LBR_VAL_F32, .valid = true};
    lbr_value_table_offer(&values, &fuel, 1000);
    lbr_can_frame_t hb;
    lbr_can_encode_heartbeat(LBR_NODE_STRESS, LBR_NODE_DEGRADED, LBR_HB_ERR_SENSOR, &hb);
    lbr_node_table_on_frame(&nodes, &hb, 1000);

    uint8_t pkt[LBR_PKT_MAX_SIZE];
    size_t  n =
        lbr_dash_link_build(&link, 1500000, LBR_NODE_CENTRAL, &values, &nodes, pkt, sizeof pkt);
    CHECK(n > 0);

    /* Dashboard side. */
    dash_fake_t            df   = {.mono = 50};
    lbr_sensor_t           batt = {.name      = "battery",
                                   .signals   = k_dash_batt,
                                   .n_signals = 1,
                                   .init      = ok_init,
                                   .read      = dash_batt_read};
    lbr_acq_t              acq  = {&batt, 1};
    lbr_node_config_t      cfg  = {.node_id = LBR_NODE_DASHBOARD, .acq = &acq, .n_acq = 1};
    lbr_dash_hal_t         hal  = {.ctx = &df, .mono_us = df_mono, .send = df_send};
    static lbr_dash_core_t d;
    lbr_dash_core_init(&d, &cfg, &hal);
    lbr_dash_core_on_packet(&d, pkt, n, df.mono);
    CHECK_EQ_U(d.packets, 1u);
    CHECK(lbr_dash_core_link_up(&d, df.mono + 1000));
    CHECK(!lbr_dash_core_link_up(&d, df.mono + LBR_WLINK_LINK_TIMEOUT_US + 1));

    lbr_value_table_t shown;
    lbr_dash_node_t   shown_nodes[LBR_DASH_MAX_NODES];
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    lbr_sample_t s;
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_ENGINE_RPM, df.mono, &s), LBR_VALUE_OK);
    CHECK(s.v.f32 == 3600.0f);
    /* Stale at the Central: shown as NO_DATA, never as the old number. */
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_FUEL_LEVEL, df.mono, &s), LBR_VALUE_OK);
    n = lbr_dash_link_build(&link, 1000 + 2000001, LBR_NODE_CENTRAL, &values, &nodes, pkt,
                            sizeof pkt);
    lbr_dash_core_on_packet(&d, pkt, n, df.mono + 10);
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_FUEL_LEVEL, df.mono + 10, &s),
               LBR_VALUE_INVALID);
    CHECK_EQ_U(s.v.u32, LBR_UNAVAIL_NO_DATA);
    CHECK_EQ_U(shown_nodes[0].node_id, LBR_NODE_STRESS);
    CHECK_EQ_U(shown_nodes[0].state, LBR_NODE_DEGRADED);

    /* The dashboard reports its battery; the Central takes it. */
    lbr_dash_core_slot_start(&d, 0);
    df.mono = 1000000;
    lbr_dash_core_slot_tick(&d, 0);
    CHECK(df.len > 0);
    lbr_dash_link_on_packet(&link, df.pkt, df.len, 5000000);
    CHECK(link.have_battery && link.battery.v.f32 == 91.0f);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000001), 1);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000002), 0);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000000 + LBR_WLINK_LINK_TIMEOUT_US + 1), 0);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000000 + LBR_WLINK_DASH_TIMEOUT_US + 1), -1);

    /* A datagram that is not from the Central is rejected. */
    lbr_dash_core_on_packet(&d, df.pkt, df.len, 0);
    CHECK_EQ_U(d.rejected, 1u);

    /* Reordered: an older datagram arriving after a newer one is ignored,
     * so the display never steps back to the older value. */
    uint8_t old_pkt[LBR_PKT_MAX_SIZE];
    rpm.v.f32 = 3000.0f;
    lbr_value_table_offer(&values, &rpm, 3000000);
    size_t old_n = lbr_dash_link_build(&link, 3000000, LBR_NODE_CENTRAL, &values, &nodes, old_pkt,
                                       sizeof old_pkt);
    rpm.v.f32    = 4000.0f;
    lbr_value_table_offer(&values, &rpm, 3100000);
    n = lbr_dash_link_build(&link, 3100000, LBR_NODE_CENTRAL, &values, &nodes, pkt, sizeof pkt);
    uint32_t packets = d.packets;
    lbr_dash_core_on_packet(&d, pkt, n, df.mono + 100);
    lbr_dash_core_on_packet(&d, old_pkt, old_n, df.mono + 200);
    CHECK_EQ_U(d.packets, packets + 1u);
    CHECK_EQ_U(d.packet_gaps, 0u);
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_ENGINE_RPM, df.mono + 200, &s), LBR_VALUE_OK);
    CHECK(s.v.f32 == 4000.0f);
}

int main(void)
{
    RUN(test_seq_window);
    RUN(test_dashboard_round_trip);
    TEST_MAIN_END();
}
