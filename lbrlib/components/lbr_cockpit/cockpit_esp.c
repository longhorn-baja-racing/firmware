#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/board.h"
#include "lbr/cockpit_app.h"
#include "lbr/cockpit_core.h"
#include "lbr/wlink.h"
#include "lbr/wlink_esp.h"
#include "sdkconfig.h"

/*
 * The cockpit role (ESP32-C5, own battery): it joins the Central's access
 * point, shows what the Central sends (live values, node status) through the
 * app's display driver, and reports its own slots (its battery) back. No CAN.
 * All decisions are in lbr/cockpit_core.h, which the simulator runs unchanged.
 */

static const char *TAG = "cockpit";

#define DEFAULT_RENDER_MS 500u
#define ACQ_TASK_PRIO     6
#define DISPLAY_TASK_PRIO 4

static lbr_sensor_t       s_sensors[LBR_NODE_MAX_SENSORS];
static lbr_acq_t          s_acq[LBR_NODE_MAX_SENSORS];
static lbr_node_config_t  s_cfg;
static lbr_cockpit_core_t s_core;
static SemaphoreHandle_t  s_lock;
static lbr_udp_t         *s_udp;

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

static void on_datagram(lbr_udp_t *self, const uint8_t *buf, size_t len, uint32_t ip, uint16_t port,
                        uint64_t rx_local_us, void *ctx)
{
    (void)self;
    (void)ip;
    (void)port;
    (void)ctx;
    lbr_cockpit_core_on_packet(&s_core, buf, len, rx_local_us);
}

/* One slot (the battery): sampled at its rate, reported to the Central. */
static void acq_task(void *arg)
{
    size_t i = (size_t)(uintptr_t)arg;
    lbr_cockpit_core_slot_start(&s_core, i);
    for (;;) {
        uint64_t due = lbr_cockpit_core_slot_next_due(&s_core, i);
        uint64_t now = lbr_time_mono_us();
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((due > now ? due - now : 0u) / 1000u) + 1u));
        lbr_cockpit_core_slot_tick(&s_core, i);
    }
}

static void display_task(void *arg)
{
    const lbr_display *d = arg;
    /* Render from a snapshot: drawing is slow and must not hold up reception. */
    static lbr_value_table_t  values;
    static lbr_cockpit_node_t nodes[LBR_COCKPIT_MAX_NODES];
    uint32_t                  period = d->period_ms != 0 ? d->period_ms : DEFAULT_RENDER_MS;
    if (d->start != NULL) {
        d->start();
    }
    ESP_LOGI(TAG, "display: %s", d->panel != NULL ? d->panel : "?");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(period));
        uint64_t now = lbr_time_mono_us();
        lbr_cockpit_core_snapshot(&s_core, &values, nodes);
        d->render(&values, nodes, lbr_cockpit_core_link_up(&s_core, now), now);
    }
}

void lbr_cockpit_main(const lbr_cockpit_app_t *app)
{
    lbr_board_print_mark();
    s_lock = xSemaphoreCreateMutex();

    const lbr_node_plan_t *plan = app->plan;
    if (plan->n_slots > LBR_NODE_MAX_SENSORS) {
        ESP_LOGE(TAG, "plan has more slots than LBR_NODE_MAX_SENSORS: none started");
        s_cfg = (lbr_node_config_t){.node_id = plan->node_id};
    } else {
        lbr_node_build(plan, LBR_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);
        lbr_node node = {.plan = plan, .cfg = &s_cfg, .sensors = s_sensors};
        for (size_t i = 0; i < plan->n_slots; i++) {
            if (plan->slots[i].input == LBR_INPUT_IO) {
                lbr_bind_io(&node, i, plan->slots[i].io_spec); /* e.g. the battery divider */
            }
        }
    }
    lbr_cockpit_hal_t hal = {
        .mono_us = hal_mono,
        .send    = hal_send,
        .lock    = hal_lock,
        .unlock  = hal_unlock,
        .log     = hal_log,
    };
    lbr_cockpit_core_init(&s_core, &s_cfg, &hal);

    if (!lbr_wifi_start_sta(CONFIG_LBR_WIFI_SSID, CONFIG_LBR_WIFI_PASSWORD) ||
        (s_udp = lbr_udp_open(LBR_WLINK_COCKPIT_PORT, on_datagram, NULL)) == NULL) {
        ESP_LOGE(TAG, "no link to the Central: the display shows no data");
    }
    for (size_t i = 0; i < s_cfg.n_acq; i++) {
        char name[16];
        snprintf(name, sizeof name, "acq_%u", (unsigned)i);
        xTaskCreate(acq_task, name, 4096, (void *)(uintptr_t)i, ACQ_TASK_PRIO, NULL);
    }
    if (app->display != NULL && app->display->render != NULL) {
        xTaskCreate(display_task, "cockpit_display", 4096, (void *)app->display, DISPLAY_TASK_PRIO,
                    NULL);
    } else {
        ESP_LOGW(TAG, "no display driver: the cockpit only reports its battery");
    }
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
