#include "lbr/health.h"
#include "lbr_test.h"

static lbr_health_record_t sample_record(void)
{
    lbr_health_record_t h = {
        .node_id               = 0x03,
        .state                 = LBR_NODE_DEGRADED,
        .flags                 = LBR_HF_TIME_SYNCED | LBR_HF_CAN_UP,
        .fw_version            = 0x000201,
        .protocol_version      = 2,
        .boot_count            = 0xBEEF,
        .catalog_hash          = 0x12345678,
        .config_crc            = 0x9ABCDEF0,
        .uptime_ms             = 0xFFFFFFF0,
        .reset_reason          = LBR_RESET_BROWNOUT,
        .acquire_hz            = 1000,
        .publish_hz            = 250,
        .log_hz                = 0,
        .queue_high_water      = 9876,
        .overflow_count        = 1,
        .rejected_count        = 2,
        .write_error_count     = 3,
        .transport_error_count = 4,
        .can_tec               = 128,
        .can_rec               = 7,
        .can_bus_off_count     = 5,
        .can_tx_dropped        = 6,
        .can_rx_dropped        = 7,
        .can_bus_errors        = 8,
        .sensor_status         = 0x5,
        .calibration_state     = 0x1,
        .last_sample_age_ms    = 12,
        .last_sd_write_age_ms  = LBR_AGE_NONE,
        .time_step_count       = 9,
        .last_sync_error_us    = -1234,
    };
    return h;
}

static void test_record_roundtrip(void)
{
    lbr_health_record_t h = sample_record();
    uint8_t             buf[LBR_HEALTH_WIRE_SIZE];
    CHECK_EQ_U(lbr_health_encode(&h, buf, sizeof buf), LBR_HEALTH_WIRE_SIZE);
    lbr_health_record_t o;
    CHECK_EQ_U(lbr_health_decode(buf, sizeof buf, &o), LBR_HEALTH_WIRE_SIZE);
    CHECK(memcmp(&h, &o, sizeof h) == 0);
    CHECK_EQ_U(lbr_health_encode(&h, buf, sizeof buf - 1), 0u);
}

static void test_page_independent(void)
{
    lbr_health_record_t h = sample_record();
    lbr_health_record_t o = {0};
    uint8_t             page[8];
    /* Pages decode independently and in any order. */
    for (int p = (int)LBR_HEALTH_PAGE_COUNT - 1; p >= 0; p--) {
        CHECK(lbr_health_page_encode(&h, (unsigned)p, page));
        CHECK(lbr_health_page_decode(&o, (unsigned)p, page));
    }
    o.node_id = h.node_id;
    CHECK(memcmp(&h, &o, sizeof h) == 0);
    CHECK(!lbr_health_page_encode(&h, LBR_HEALTH_PAGE_COUNT, page));
}

static void test_reserved_rejected(void)
{
    lbr_health_record_t h = sample_record();
    uint8_t             page[8];
    lbr_health_page_encode(&h, 3, page);
    page[2] = 1;
    CHECK(!lbr_health_page_decode(&h, 3, page));
    lbr_health_page_encode(&h, 9, page);
    page[7] = 1;
    CHECK(!lbr_health_page_decode(&h, 9, page));
}

static void test_helpers(void)
{
    uint32_t hw = 0;
    lbr_high_water_update(&hw, 50, 100);
    CHECK_EQ_U(hw, 5000u);
    lbr_high_water_update(&hw, 10, 100);
    CHECK_EQ_U(hw, 5000u);
    CHECK_EQ_U(lbr_age_ms(5000000, 0), LBR_AGE_NONE);
    CHECK_EQ_U(lbr_age_ms(5000000, 2000000), 3000u);
    CHECK_EQ_U(lbr_age_ms(1000, 2000), 0u);
}

int main(void)
{
    RUN(test_record_roundtrip);
    RUN(test_page_independent);
    RUN(test_reserved_rejected);
    RUN(test_helpers);
    TEST_MAIN_END();
}
