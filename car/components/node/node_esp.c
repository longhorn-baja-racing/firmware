#include <stdio.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/can_bus.h"
#include "lbr/log_service.h"
#include "lbr/node.h"
#include "lbr/node_core.h"
#include "lbr/sensor_io.h"
#include "lbr/timebase.h"
#include "sdkconfig.h"

/*
 * ESP32 platform for the sensor-node runtime: FreeRTOS tasks, esp_timer
 * wake-ups on the shared time grid, TWAI, the SD log service, and the
 * watchdog. All decisions are in node_core.c, which the simulator runs
 * unchanged.
 */

static const char *TAG = "lbr_node";

#define ACQ_TASK_PRIO    10
#define RX_TASK_PRIO     12
#define HEALTH_TASK_PRIO 6

typedef struct {
    size_t             index;
    TaskHandle_t       task;
    esp_timer_handle_t timer;
} acq_task_t;

static lbr_node_core_t   s_core;
static bool              s_started;
static acq_task_t        s_acq[LBR_NODE_MAX_SENSORS];
static SemaphoreHandle_t s_evt_lock;

/* ------------------------------------------------------------------------ */
/* HAL                                                                       */

static uint64_t hal_mono_us(void *ctx)
{
    (void)ctx;
    return lbr_time_mono_us();
}

static uint64_t hal_now_us(void *ctx)
{
    (void)ctx;
    return lbr_time_now_us();
}

static uint64_t hal_local_to_shared(void *ctx, uint64_t local_us)
{
    (void)ctx;
    return lbr_time_local_to_shared(local_us);
}

static bool hal_time_synced(void *ctx)
{
    (void)ctx;
    return lbr_time_is_synced();
}

static lbr_time_update_t hal_time_apply(void *ctx, uint64_t gw_us, uint64_t local_us,
                                        uint16_t epoch)
{
    (void)ctx;
    return lbr_time_sync_apply(gw_us, local_us, epoch);
}

static void hal_time_servo(void *ctx, lbr_time_servo_t *out)
{
    (void)ctx;
    lbr_time_get_servo(out);
}

static bool hal_can_send(void *ctx, const lbr_can_frame_t *f)
{
    (void)ctx;
    return lbr_can_bus_send(f);
}

static void hal_can_stats(void *ctx, lbr_can_stats_t *out)
{
    (void)ctx;
    lbr_can_bus_get_stats(out);
}

static bool hal_sd_present(void *ctx)
{
    (void)ctx;
    return lbr_log_service_sd_present();
}

static void hal_lock(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_evt_lock, portMAX_DELAY);
}

static void hal_unlock(void *ctx)
{
    (void)ctx;
    xSemaphoreGive(s_evt_lock);
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

static const lbr_node_hal_t k_hal = {
    .mono_us         = hal_mono_us,
    .now_us          = hal_now_us,
    .local_to_shared = hal_local_to_shared,
    .time_synced     = hal_time_synced,
    .time_apply      = hal_time_apply,
    .time_servo      = hal_time_servo,
    .can_send        = hal_can_send,
    .can_stats       = hal_can_stats,
    .sd_present      = hal_sd_present,
    .lock            = hal_lock,
    .unlock          = hal_unlock,
    .log             = hal_log,
};

struct lbr_node_core *lbr_node_running_core(void)
{
    return s_started ? &s_core : NULL;
}

void lbr_node_emit_event(lbr_event_code_t code, uint16_t arg, bool fault)
{
    if (!s_started) {
        ESP_LOGW(TAG, "%s 0x%04x arg %u (before start)", fault ? "FAULT" : "event", (unsigned)code,
                 (unsigned)arg);
        return;
    }
    lbr_node_core_event(&s_core, code, arg, fault);
}

/* ------------------------------------------------------------------------ */
/* Tasks                                                                     */

static void acq_timer_cb(void *arg)
{
    acq_task_t *t = arg;
    xTaskNotifyGive(t->task);
}

/* Sample k is due at shared time k * period: wake with a one-shot timer at
 * the next grid point, recomputed every time, so the slot stays locked to
 * the shared clock through slews and steps. */
static void acq_task(void *arg)
{
    acq_task_t *t = arg;
    esp_task_wdt_add(NULL);
    lbr_node_core_slot_start(&s_core, t->index);
    for (;;) {
        uint64_t due = lbr_node_core_slot_next_due(&s_core, t->index);
        uint64_t now = lbr_time_now_us();
        esp_timer_start_once(t->timer, due > now ? due - now : 1u);
        uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        esp_task_wdt_reset();
        if (woke > 0) {
            lbr_node_core_slot_tick(&s_core, t->index);
        } else {
            esp_timer_stop(t->timer);
        }
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    lbr_can_rx_t rx;
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        if (lbr_can_bus_receive(&rx, 500)) {
            lbr_node_core_on_frame(&s_core, &rx.frame, rx.rx_local_us);
        }
    }
}

static void health_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(s_core.cfg->health_period_ms));
        esp_task_wdt_reset();
        lbr_node_core_heartbeat(&s_core, NULL);
    }
}

/* ------------------------------------------------------------------------ */
/* Entry                                                                     */

static void on_log_event(uint16_t code, bool fault)
{
    lbr_node_core_on_log_event(&s_core, code, fault);
}

void lbr_node_bind_io(const lbr_node_plan_t *plan, size_t slot, lbr_sensor_t *sensors,
                      const char *spec)
{
    if (spec == NULL || spec[0] == '\0' || slot >= plan->n_slots) {
        return;
    }
    const lbr_slot_t *s = &plan->slots[slot];
    lbr_sensor_io(&sensors[slot], s->name, s->signals, s->n_signals, spec);
}

void lbr_node_run(const lbr_node_config_t *cfg)
{
    s_evt_lock                   = xSemaphoreCreateMutex();
    lbr_can_bus_config_t can_cfg = lbr_can_bus_default_config();
    lbr_node_boot_t      boot    = {
        .reset_reason = lbr_health_reset_reason(),
        .boot_count   = lbr_health_boot_count(),
        .can_bitrate  = can_cfg.bitrate,
    };
    bool valid = lbr_node_core_init(&s_core, cfg, &k_hal, &boot);

    can_cfg.accept_all = cfg->listen_all;
    lbr_can_err_t can  = lbr_can_bus_start(&can_cfg);

    lbr_logger_t *log = NULL;
    if (cfg->local_log) {
        lbr_log_service_config_t lc = {
            .node_id          = cfg->node_id,
            .reset_reason     = (uint8_t)boot.reset_reason,
            .boot_count       = boot.boot_count,
            .config_text      = s_core.config_text,
            .queue_size       = (size_t)CONFIG_LBR_LOG_QUEUE_KB * 1024u,
            .sync_interval_ms = CONFIG_LBR_LOG_SYNC_INTERVAL_MS,
            .now_us           = lbr_time_now_us,
            .on_event         = on_log_event,
        };
        if (lbr_log_service_start(&lc)) {
            log = lbr_log_service_logger();
        } else {
            ESP_LOGE(TAG, "local log init failed (out of memory)");
        }
    }

    lbr_node_core_start(&s_core, can == LBR_CAN_OK, can == LBR_CAN_ERR_PINS_UNSET, log);
    s_started = true;
    if (cfg->on_started != NULL) {
        cfg->on_started(cfg->on_started_ctx);
    }

    if (valid) {
        for (size_t i = 0; i < cfg->n_acq && i < LBR_NODE_MAX_SENSORS; i++) {
            acq_task_t *t                 = &s_acq[i];
            t->index                      = i;
            esp_timer_create_args_t targs = {
                .callback = acq_timer_cb,
                .arg      = t,
                .name     = "lbr_acq",
            };
            char name[16];
            snprintf(name, sizeof name, "acq_%u", (unsigned)i);
            /* Timer first: the task arms it as soon as it runs. */
            if (esp_timer_create(&targs, &t->timer) != ESP_OK ||
                xTaskCreate(acq_task, name, 4096, t, ACQ_TASK_PRIO, &t->task) != pdPASS) {
                ESP_LOGE(TAG, "cannot start acquisition for %s", cfg->acq[i].sensor->name);
            }
        }
    }

    if (s_core.can_up) {
        xTaskCreate(rx_task, "lbr_rx", 4096, NULL, RX_TASK_PRIO, NULL);
    }
    xTaskCreate(health_task, "lbr_health", 4096, NULL, HEALTH_TASK_PRIO, NULL);

    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
