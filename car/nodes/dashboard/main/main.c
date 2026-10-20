#include <stdio.h>

#include "dashboard_plan.h"
#include "display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/dash_core.h"
#include "lbr/node_app.h"
#include "lbr/sensor_io.h"
#include "lbr/timebase.h"
#include "lbr/wlink.h"
#include "lbr/wlink_esp.h"
#include "sdkconfig.h"

/*
 * Cockpit dashboard app (ESP32-C5, own battery). It joins the Central's
 * access point, shows what the Central sends (live values, node status), and
 * reports its battery back (lbr/dash_core.h, run unchanged by the
 * simulator). No sensors on the car, no CAN.
 */

static const char *TAG = "dash";

#define RENDER_PERIOD_MS 500
#define ACQ_TASK_PRIO    6

static lbr_sensor_t      s_sensors[DASHBOARD_SLOT_COUNT];
static lbr_acq_t         s_acq[DASHBOARD_SLOT_COUNT];
static lbr_node_config_t s_cfg;
static lbr_dash_core_t   s_core;
static SemaphoreHandle_t s_lock;
static lbr_udp_t        *s_udp;

static uint64_t hal_mono(void *ctx)
{
    (void)ctx;
    return lbr_time_mono_us();
}

static bool hal_send(void *ctx, const uint8_t *pkt, size_t len)
{
    (void)ctx;
    return lbr_udp_send(s_udp, lbr_wifi_sta_gateway(), LBR_WLINK_STATUS_PORT, pkt, len);
}

static void hal_lock(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void hal_unlock(void *ctx)
{
    (void)ctx;
    xSemaphoreGive(s_lock);
}

static void hal_log(void *ctx, lbr_log_level_t level, const char *text)
{
    (void)ctx;
    if (level == LBR_LOG_LVL_ERROR) {
        ESP_LOGE(TAG, "%s", text);
    } else {
        ESP_LOGI(TAG, "%s", text);
    }
}

static void on_datagram(const uint8_t *buf, size_t len, uint32_t ip, uint16_t port,
                        uint64_t rx_local_us, void *ctx)
{
    (void)ip;
    (void)port;
    (void)ctx;
    lbr_dash_core_on_packet(&s_core, buf, len, rx_local_us);
}

/* The battery slot: once a second, reported to the Central. */
static void acq_task(void *arg)
{
    size_t i = (size_t)(uintptr_t)arg;
    lbr_dash_core_slot_start(&s_core, i);
    for (;;) {
        uint64_t due = lbr_dash_core_slot_next_due(&s_core, i);
        uint64_t now = lbr_time_mono_us();
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((due > now ? due - now : 0u) / 1000u) + 1u));
        lbr_dash_core_slot_tick(&s_core, i);
    }
}

static void display_task(void *arg)
{
    (void)arg;
    /* Render from a snapshot: drawing is slow and must not hold up reception. */
    static lbr_value_table_t values;
    static lbr_dash_node_t   nodes[LBR_DASH_MAX_NODES];
    dash_display_init();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RENDER_PERIOD_MS));
        uint64_t now = lbr_time_mono_us();
        lbr_dash_core_snapshot(&s_core, &values, nodes);
        dash_display_render(&values, nodes, lbr_dash_core_link_up(&s_core, now), now);
    }
}

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    s_lock = xSemaphoreCreateMutex();
    lbr_node_build(&lbr_plan_dashboard, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);
    /* Battery: a divider on an ADC pin (CONFIG_LBR_IO_BATTERY), part TBD. */
    if (CONFIG_LBR_IO_BATTERY[0] != '\0') {
        const lbr_slot_t *b = &lbr_plan_dashboard.slots[DASHBOARD_SLOT_BATTERY];
        lbr_sensor_io(&s_sensors[DASHBOARD_SLOT_BATTERY], b->name, b->signals, b->n_signals,
                      CONFIG_LBR_IO_BATTERY);
    }
    lbr_dash_hal_t hal = {
        .mono_us = hal_mono,
        .send    = hal_send,
        .lock    = hal_lock,
        .unlock  = hal_unlock,
        .log     = hal_log,
    };
    lbr_dash_core_init(&s_core, &s_cfg, &hal);

    if (!lbr_wifi_start_sta(CONFIG_LBR_WIFI_SSID, CONFIG_LBR_WIFI_PASSWORD) ||
        (s_udp = lbr_udp_open(LBR_WLINK_DASH_PORT, on_datagram, NULL)) == NULL) {
        ESP_LOGE(TAG, "no link to the Central: the display shows no data");
    }
    for (size_t i = 0; i < s_cfg.n_acq; i++) {
        char name[16];
        snprintf(name, sizeof name, "acq_%u", (unsigned)i);
        xTaskCreate(acq_task, name, 4096, (void *)(uintptr_t)i, ACQ_TASK_PRIO, NULL);
    }
    xTaskCreate(display_task, "dash_display", 4096, NULL, 4, NULL);
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
