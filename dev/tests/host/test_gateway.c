#include "lbr/can.h"
#include "lbr/gw_config.h"
#include "lbr/nodes.h"
#include "lbr/status.h"
#include "lbr_test.h"

static void test_config_defaults_roundtrip(void)
{
    lbr_gw_config_t c, d;
    char            text[512];
    char            err[96];
    lbr_gw_config_defaults(&c);
    size_t n = lbr_gw_config_format(&c, text, sizeof text);
    CHECK(n > 0);
    CHECK_EQ_I(lbr_gw_config_parse(text, n, &d, err, sizeof err), 0);
    CHECK(memcmp(&c, &d, sizeof c) == 0);
}

static void test_config_parse(void)
{
    const char     *t = "# pit tune\n lora_period_ms = 500 \r\nble_enable=0\n\n";
    lbr_gw_config_t c;
    char            err[96] = "";
    CHECK_EQ_I(lbr_gw_config_parse(t, strlen(t), &c, err, sizeof err), 0);
    CHECK_EQ_U(c.lora_period_ms, 500);
    CHECK(!c.ble_enable);
    CHECK(c.wifi_enable); /* unspecified => default */
}

static void test_config_rejects(void)
{
    const char *bad[] = {
        "lora_period_ms=10\n",  /* below range */
        "lora_period_ms=abc\n", /* not a number */
        "usb_enable=2\n",       /* bool out of range */
        "unknown_key=1\n",      /* unknown key */
        "lora_enable\n",        /* no '=' */
        "node_timeout_ms=-5\n", /* negative */
        "stream_batch_ms=\n",   /* empty */
        "lora_period_ms=99999999\n",
    };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        lbr_gw_config_t c;
        lbr_gw_config_defaults(&c);
        c.lora_period_ms = 1234;
        char err[96]     = "";
        CHECK(lbr_gw_config_parse(bad[i], strlen(bad[i]), &c, err, sizeof err) < 0);
        CHECK(err[0] != '\0');
        CHECK_EQ_U(c.lora_period_ms, 1234); /* untouched on failure */
    }
    const char      bin[] = "lora_enable=1\x01\n";
    lbr_gw_config_t c;
    CHECK(lbr_gw_config_parse(bin, sizeof bin - 1, &c, NULL, 0) < 0);
}

static void test_status_text(void)
{
    lbr_node_table_t t;
    lbr_node_table_init(&t, 3000000);
    lbr_health_record_t h = {.state         = LBR_NODE_RUN,
                             .flags         = LBR_HF_TIME_SYNCED | LBR_HF_SIM_SENSORS,
                             .uptime_ms     = 120000,
                             .sensor_status = 0x1F,
                             .fw_version    = 0x000200};
    lbr_can_frame_t     f;
    for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
        lbr_can_encode_health_page(LBR_NODE_IMU_GPS, &h, p, &f);
        lbr_node_table_on_frame(&t, &f, 1000);
    }
    lbr_can_encode_health_page(LBR_NODE_STRESS, &h, 0, &f);
    lbr_node_table_on_frame(&t, &f, 1000);
    uint8_t lost[4];
    lbr_node_table_expire(&t, 1000 + 3000001, lost, 4);

    lbr_health_record_t gw = {
        .state = LBR_NODE_RUN, .flags = LBR_HF_TIME_SYNCED | LBR_HF_SD_PRESENT, .uptime_ms = 5000};
    char   buf[512];
    size_t n = lbr_status_format(&gw, "00000012.LBR", &t, 15000000, buf, sizeof buf);
    CHECK(n > 0 && n < sizeof buf);
    CHECK(strstr(buf, "gw RUN sync sd up 5s log=00000012.LBR") != NULL);
    CHECK(strstr(buf, "imu_gps LOST") != NULL);
    CHECK(strstr(buf, "stress LOST 14s") != NULL);
    for (size_t i = 0; i < n; i++) {
        CHECK((unsigned char)buf[i] < 0x80);
    }
    char small[10];
    CHECK_EQ_U(lbr_status_format(&gw, NULL, &t, 0, small, sizeof small), 9u);
}

int main(void)
{
    RUN(test_config_defaults_roundtrip);
    RUN(test_config_parse);
    RUN(test_config_rejects);
    RUN(test_status_text);
    TEST_MAIN_END();
}
