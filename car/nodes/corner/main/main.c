#include <stdio.h>

#include "corner_plan.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lbr/corner_core.h"
#include "lbr/node_app.h"
#include "lbr/sensor_io.h"
#include "lbr/timebase.h"
#include "lbr/wlink.h"
#include "lbr/wlink_esp.h"
#include "sdkconfig.h"

/*
 * Corner suspension node app (Mini ESP32-C5 on the moving suspension, own
 * battery). The same firmware runs on all four; CONFIG_LBR_CORNER_POSITION
 * picks the plan. It joins the IMU Controller's access point and sends its
 * IMU samples there (lbr/corner_core.h, run unchanged by the simulator). No
 * CAN, no SD card.
 */

static const char *TAG = "corner";

#define ACQ_TASK_PRIO   10
#define CLOCK_TASK_PRIO 9

typedef struct {
    size_t             index;
    TaskHandle_t       task;
    esp_timer_handle_t timer;
} acq_task_t;

static lbr_sensor_t      s_sensors[CORNER_SLOT_COUNT];
static lbr_acq_t         s_acq[CORNER_SLOT_COUNT];
static lbr_node_config_t s_cfg;
static lbr_corner_core_t s_core;
static acq_task_t        s_tasks[CORNER_SLOT_COUNT];
static lbr_udp_t        *s_udp;

/* ------------------------------------------------------------------------ */
/* Platform                                                                  */

static uint64_t hal_mono(void *ctx)
{
    (void)ctx;
    return lbr_time_mono_us();
}

static uint64_t hal_now(void *ctx)
{
    (void)ctx;
    return lbr_time_now_us();
}

static void hal_servo(void *ctx, lbr_time_servo_t *out)
{
    (void)ctx;
    lbr_time_get_servo(out);
}

static lbr_time_update_t hal_apply(void *ctx, uint64_t shared, uint64_t local, uint16_t epoch)
{
    (void)ctx;
    return lbr_time_sync_apply(shared, local, epoch);
}

static bool hal_send(void *ctx, const uint8_t *pkt, size_t len)
{
    (void)ctx;
    return lbr_udp_send(s_udp, lbr_wifi_sta_gateway(), LBR_WLINK_CORNER_PORT, pkt, len);
}

static void hal_log(void *ctx, lbr_log_level_t level, const char *text)
{
    (void)ctx;
    if (level == LBR_LOG_LVL_ERROR) {
        ESP_LOGE(TAG, "%s", text);
    } else if (level == LBR_LOG_LVL_WARN) {
        ESP_LOGW(TAG, "%s", text);
    } else {
        ESP_LOGI(TAG, "%s", text);
    }
}

static const lbr_corner_hal_t k_hal = {
    .mono_us    = hal_mono,
    .now_us     = hal_now,
    .time_servo = hal_servo,
    .time_apply = hal_apply,
    .send       = hal_send,
    .log        = hal_log,
};

/* ------------------------------------------------------------------------ */
/* Tasks                                                                     */

static void acq_timer_cb(void *arg)
{
    acq_task_t *t = arg;
    xTaskNotifyGive(t->task);
}

/* Sample on the shared time grid, as node_esp.c does. */
static void acq_task(void *arg)
{
    acq_task_t *t = arg;
    esp_task_wdt_add(NULL);
    lbr_corner_core_slot_start(&s_core, t->index);
    for (;;) {
        uint64_t due = lbr_corner_core_slot_next_due(&s_core, t->index);
        uint64_t now = lbr_time_now_us();
        esp_timer_start_once(t->timer, due > now ? due - now : 1u);
        uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        esp_task_wdt_reset();
        if (woke > 0) {
            lbr_corner_core_slot_tick(&s_core, t->index);
        } else {
            esp_timer_stop(t->timer);
        }
    }
}

static void clock_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t ms = lbr_corner_core_clock_tick(&s_core);
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static void on_datagram(const uint8_t *buf, size_t len, uint32_t ip, uint16_t port,
                        uint64_t rx_local_us, void *ctx)
{
    (void)ip;
    (void)port;
    (void)ctx;
    lbr_corner_core_on_packet(&s_core, buf, len, rx_local_us);
}

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    const lbr_node_plan_t *plan = &lbr_plan_corner[CONFIG_LBR_CORNER_POSITION];
    lbr_node_build(plan, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);

    /*
     * The corner IMU waits for a part-specific driver (issue #10):
     *
     *     lbr_sensor_<part>(&s_sensors[CORNER_SLOT_IMU], ...);
     *
     * The battery is usually a divider on an ADC pin (CONFIG_LBR_IO_BATTERY).
     */
    if (CONFIG_LBR_IO_BATTERY[0] != '\0') {
        const lbr_slot_t *b = &plan->slots[CORNER_SLOT_BATTERY];
        lbr_sensor_io(&s_sensors[CORNER_SLOT_BATTERY], b->name, b->signals, b->n_signals,
                      CONFIG_LBR_IO_BATTERY);
    }
    if (!lbr_corner_core_init(&s_core, &s_cfg, &k_hal, LBR_CORNER_DEFAULT_BATCH)) {
        ESP_LOGE(TAG, "configuration rejected: nothing will be sent");
    }

    /* WiFi first: the clock and the samples go nowhere without it, but the
     * node samples anyway (the IMU Controller counts what never arrived). */
    if (!lbr_wifi_start_sta(CONFIG_LBR_CORNER_SSID, CONFIG_LBR_CORNER_PASSWORD) ||
        (s_udp = lbr_udp_open(LBR_WLINK_CORNER_PORT, on_datagram, NULL)) == NULL) {
        ESP_LOGE(TAG, "corner network unavailable");
    }

    for (size_t i = 0; i < s_cfg.n_acq; i++) {
        acq_task_t *t                 = &s_tasks[i];
        t->index                      = i;
        esp_timer_create_args_t targs = {.callback = acq_timer_cb, .arg = t, .name = "corner_acq"};
        char                    name[16];
        snprintf(name, sizeof name, "acq_%u", (unsigned)i);
        if (esp_timer_create(&targs, &t->timer) != ESP_OK ||
            xTaskCreate(acq_task, name, 4096, t, ACQ_TASK_PRIO, &t->task) != pdPASS) {
            ESP_LOGE(TAG, "cannot start slot %s", s_acq[i].sensor->name);
        }
    }
    xTaskCreate(clock_task, "corner_clock", 3072, NULL, CLOCK_TASK_PRIO, NULL);
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
