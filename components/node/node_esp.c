#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/can_bus.h"
#include "lbr/crc32.h"
#include "lbr/log_service.h"
#include "lbr/node.h"
#include "lbr/nodes.h"
#include "lbr/timebase.h"
#include "lbr/version.h"
#include "sdkconfig.h"

static const char *TAG = "lbr_node";

#define ACQ_TASK_PRIO    10
#define RX_TASK_PRIO     12
#define HEALTH_TASK_PRIO 6
#define LOG_TASK_PRIO    5
#define SENSOR_RETRY_US  2000000u
#define SD_RETRY_US      5000000u
#define LOG_PUMP_MS      20
#define LOG_CHUNK_SIZE   8192
#define CONFIG_TEXT_MAX  1536

typedef struct {
    const lbr_acq_t   *acq;
    uint8_t            index;
    TaskHandle_t       task;
    esp_timer_handle_t timer;
    lbr_stream_t       streams[LBR_SENSOR_MAX_SIGNALS];
    atomic_bool        healthy;
    atomic_bool        calibrated;
} acq_ctx_t;

static const lbr_node_config_t *s_cfg;
static acq_ctx_t                s_acq[LBR_NODE_MAX_SENSORS];
static bool                     s_config_valid;
static bool                     s_can_up;
static bool                     s_sim;
static uint32_t                 s_config_crc;
static char                     s_config_text[CONFIG_TEXT_MAX];
static lbr_reset_reason_t       s_reset_reason;
static uint32_t                 s_boot_count;

/* Event streams and the local logger are shared between tasks. */
static SemaphoreHandle_t s_evt_lock;
static lbr_stream_t      s_evt_stream;
static lbr_stream_t      s_fault_stream;

static lbr_logger_t *s_log; /* local log (local_log nodes only) */

static atomic_uint   s_acquired;
static atomic_uint   s_published;
static atomic_uint   s_logged;
static atomic_uint   s_overruns;
static atomic_ullong s_last_sample_us;
static atomic_uint   s_sync_unmatched;

/* ------------------------------------------------------------------------ */
/* Publishing                                                                */

static void publish(const lbr_sample_t *s)
{
    lbr_can_frame_t f;
    if (s_can_up && lbr_can_encode_sample(s, &f) && lbr_can_bus_send(&f)) {
        atomic_fetch_add(&s_published, 1);
    }
    /* A failed send is counted inside the CAN adapter (tx_dropped). */
}

static void log_sample(const lbr_sample_t *s)
{
    if (s_log != NULL && lbr_log_append_sample(s_log, s) == 0) {
        atomic_fetch_add(&s_logged, 1);
    }
    /* Drops are counted by the logger (records_dropped / rejected_closed). */
}

void lbr_node_emit_event(lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_reading_t r = {.valid = true, .v.u32 = lbr_event_value(code, arg)};
    lbr_sample_t  s;
    uint32_t      pub_seq = 0;

    ESP_LOGW(TAG, "%s 0x%04x arg %u", fault ? "FAULT" : "event", (unsigned)code, (unsigned)arg);
    if (s_evt_lock == NULL || s_cfg == NULL) {
        return;
    }
    xSemaphoreTake(s_evt_lock, portMAX_DELAY);
    lbr_stream_sample(fault ? &s_fault_stream : &s_evt_stream, 1, s_cfg->node_id,
                      fault ? LBR_SIG_FAULT : LBR_SIG_EVENT, &r, lbr_time_now_us(),
                      lbr_time_is_synced(), &s, &pub_seq);
    xSemaphoreGive(s_evt_lock);

    log_sample(&s);
    s.seq = pub_seq;
    publish(&s);
}

/* ------------------------------------------------------------------------ */
/* Acquisition                                                               */

static void acq_timer_cb(void *arg)
{
    acq_ctx_t *ctx = arg;
    xTaskNotifyGive(ctx->task);
}

static void mark_all(lbr_reading_t *out, size_t n, lbr_unavail_reason_t why)
{
    for (size_t i = 0; i < n; i++) {
        out[i].valid  = false;
        out[i].reason = why;
        out[i].v.u32  = 0;
    }
}

static void acq_task(void *arg)
{
    acq_ctx_t    *ctx    = arg;
    lbr_sensor_t *sensor = ctx->acq->sensor;
    lbr_reading_t readings[LBR_SENSOR_MAX_SIGNALS];
    uint64_t      next_retry = 0;

    esp_task_wdt_add(NULL);
    bool ok = sensor->init(sensor) == 0;
    atomic_store(&ctx->healthy, ok);
    if (!ok) {
        lbr_node_emit_event(LBR_EVT_SENSOR_INIT_FAIL, ctx->index, true);
        next_retry = lbr_time_mono_us() + SENSOR_RETRY_US;
    }
    esp_timer_start_periodic(ctx->timer, 1000000u / ctx->acq->rate_hz);

    for (;;) {
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        esp_task_wdt_reset();
        if (ticks == 0) {
            continue;
        }
        if (ticks > 1) {
            /* Acquisition fell behind its schedule: missed slots are counted. */
            atomic_fetch_add(&s_overruns, ticks - 1);
        }

        uint64_t t      = lbr_time_now_us();
        bool     synced = lbr_time_is_synced();

        if (atomic_load(&ctx->healthy)) {
            if (sensor->read(sensor, t, readings) != 0) {
                atomic_store(&ctx->healthy, false);
                lbr_node_emit_event(LBR_EVT_SENSOR_INIT_FAIL, ctx->index, true);
                next_retry = lbr_time_mono_us() + SENSOR_RETRY_US;
                mark_all(readings, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
            }
        } else {
            mark_all(readings, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
            if (lbr_time_mono_us() >= next_retry) {
                if (sensor->init(sensor) == 0) {
                    atomic_store(&ctx->healthy, true);
                    lbr_node_emit_event(LBR_EVT_SENSOR_RECOVERED, ctx->index, false);
                } else {
                    next_retry = lbr_time_mono_us() + SENSOR_RETRY_US;
                }
            }
        }
        atomic_store(&ctx->calibrated,
                     sensor->is_calibrated == NULL || sensor->is_calibrated(sensor));

        for (size_t c = 0; c < sensor->n_signals; c++) {
            lbr_sample_t s;
            uint32_t     pub_seq;
            bool pub = lbr_stream_sample(&ctx->streams[c], ctx->acq->publish_div, s_cfg->node_id,
                                         sensor->signals[c], &readings[c], t, synced, &s, &pub_seq);
            log_sample(&s);
            if (pub) {
                s.seq = pub_seq;
                publish(&s);
            }
        }
        atomic_fetch_add(&s_acquired, 1);
        atomic_store(&s_last_sample_us, t);
    }
}

/* ------------------------------------------------------------------------ */
/* CAN receive: time sync + app frames                                       */

static void rx_task(void *arg)
{
    (void)arg;
    lbr_time_sync_rx_t sync = {0};
    lbr_can_rx_t       rx;

    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        if (!lbr_can_bus_receive(&rx, 500)) {
            continue;
        }
        lbr_can_id_t id = lbr_can_id_parse(rx.frame.id);
        uint8_t      seq;
        uint16_t     epoch;
        uint64_t     tx_us;

        if (id.source == LBR_NODE_GATEWAY && lbr_can_decode_time_sync(&rx.frame, &seq, &epoch)) {
            lbr_time_sync_on_sync(&sync, seq, epoch, rx.rx_local_us);
            continue;
        }
        if (id.source == LBR_NODE_GATEWAY &&
            lbr_can_decode_time_follow_up(&rx.frame, &seq, &tx_us)) {
            uint64_t gw_us, local_us;
            if (lbr_time_sync_on_follow_up(&sync, seq, tx_us, &gw_us, &local_us, &epoch)) {
                lbr_time_update_t r = lbr_time_sync_apply(gw_us, local_us, epoch);
                if (r == LBR_TIME_UPD_FIRST || r == LBR_TIME_UPD_EPOCH_CHANGE) {
                    lbr_node_emit_event(LBR_EVT_TIME_SYNCED, epoch, false);
                } else if (r == LBR_TIME_UPD_STEP) {
                    lbr_node_emit_event(LBR_EVT_TIME_STEP, 0, false);
                }
            } else {
                atomic_store(&s_sync_unmatched, sync.unmatched);
            }
            continue;
        }
        if (s_cfg->on_frame != NULL) {
            s_cfg->on_frame(&rx.frame, lbr_time_local_to_shared(rx.rx_local_us),
                            s_cfg->on_frame_ctx);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Local SD log (redundancy nodes only)                                      */

static void on_log_event(uint16_t code, bool fault)
{
    lbr_node_emit_event((lbr_event_code_t)code, 0, fault);
}

static bool start_local_log(void)
{
    lbr_log_service_config_t lc = {
        .node_id          = s_cfg->node_id,
        .reset_reason     = (uint8_t)s_reset_reason,
        .boot_count       = s_boot_count,
        .config_text      = s_config_text,
        .queue_size       = (size_t)CONFIG_LBR_LOG_QUEUE_KB * 1024u,
        .sync_interval_ms = CONFIG_LBR_LOG_SYNC_INTERVAL_MS,
        .now_us           = lbr_time_now_us,
        .on_event         = on_log_event,
    };
    if (!lbr_log_service_start(&lc)) {
        return false;
    }
    s_log = lbr_log_service_logger();
    return true;
}

/* ------------------------------------------------------------------------ */
/* Heartbeat                                                                 */

static uint16_t rate_hz(uint32_t now, uint32_t *prev, uint32_t period_ms)
{
    uint32_t d = now - *prev;
    *prev      = now;
    return (uint16_t)((uint64_t)d * 1000u / period_ms);
}

static void build_health(lbr_health_record_t *h, uint32_t period_ms)
{
    static uint32_t  prev_acq, prev_pub, prev_log;
    lbr_can_stats_t  cs;
    lbr_log_stats_t  ls = {0};
    lbr_time_servo_t ts;

    lbr_can_bus_get_stats(&cs);
    lbr_time_get_servo(&ts);
    if (s_log != NULL) {
        lbr_log_get_stats(s_log, &ls);
    }
    bool log_open = s_log != NULL && lbr_log_is_open(s_log);

    uint32_t sensor_ok  = 0;
    uint32_t calibrated = 0;
    for (size_t i = 0; i < s_cfg->n_acq; i++) {
        sensor_ok |= atomic_load(&s_acq[i].healthy) ? (1u << i) : 0u;
        calibrated |= atomic_load(&s_acq[i].calibrated) ? (1u << i) : 0u;
    }
    uint32_t all_sensors = s_cfg->n_acq >= 32 ? 0xFFFFFFFFu : (1u << s_cfg->n_acq) - 1u;

    uint16_t flags = 0;
    flags |= ts.synced ? LBR_HF_TIME_SYNCED : 0;
    flags |= (s_log != NULL && lbr_log_service_sd_present()) ? LBR_HF_SD_PRESENT : 0;
    flags |= log_open ? LBR_HF_SD_LOGGING : 0;
    flags |= s_sim ? LBR_HF_SIM_SENSORS : 0;
    flags |= s_config_valid ? LBR_HF_CONFIG_VALID : 0;
    flags |= cs.running ? LBR_HF_CAN_UP : 0;

    lbr_node_state_t state = LBR_NODE_RUN;
    if (!s_config_valid) {
        state = LBR_NODE_CONFIG_INVALID;
    } else if (!s_can_up) {
        state = LBR_NODE_FAULT;
    } else if (sensor_ok != all_sensors || !ts.synced || !cs.running ||
               (s_cfg->local_log && !log_open)) {
        state = LBR_NODE_DEGRADED;
    }

    uint64_t now = lbr_time_now_us();
    *h           = (lbr_health_record_t){
        .node_id               = s_cfg->node_id,
        .state                 = (uint8_t)state,
        .flags                 = flags,
        .fw_version            = LBR_FW_VERSION,
        .protocol_version      = LBR_PROTOCOL_VERSION,
        .boot_count            = (uint16_t)s_boot_count,
        .catalog_hash          = lbr_catalog_hash(),
        .config_crc            = s_config_crc,
        .uptime_ms             = (uint32_t)(lbr_time_mono_us() / 1000u),
        .reset_reason          = (uint8_t)s_reset_reason,
        .acquire_hz            = rate_hz(atomic_load(&s_acquired), &prev_acq, period_ms),
        .publish_hz            = rate_hz(atomic_load(&s_published), &prev_pub, period_ms),
        .log_hz                = rate_hz(atomic_load(&s_logged), &prev_log, period_ms),
        .queue_high_water      = ls.queue_high_water,
        .overflow_count        = atomic_load(&s_overruns) + ls.records_dropped +
                                 (s_cfg->local_log ? ls.records_rejected_closed : 0u),
        .rejected_count        = cs.rx_foreign + atomic_load(&s_sync_unmatched),
        .write_error_count     = ls.write_errors + ls.sync_errors,
        .transport_error_count = cs.tx_failed,
        .can_tec               = cs.tec,
        .can_rec               = cs.rec,
        .can_bus_off_count     = cs.bus_off_count,
        .can_tx_dropped        = cs.tx_dropped,
        .can_rx_dropped        = cs.rx_dropped,
        .can_bus_errors        = cs.bus_errors,
        .sensor_status         = sensor_ok,
        .calibration_state     = calibrated,
        .last_sample_age_ms    = lbr_age_ms(now, atomic_load(&s_last_sample_us)),
        .last_sd_write_age_ms = s_cfg->local_log ? lbr_age_ms(now, ls.last_write_us) : LBR_AGE_NONE,
        .time_step_count      = ts.step_count,
        .last_sync_error_us   = ts.last_error_us,
    };
}

static void health_task(void *arg)
{
    (void)arg;
    const uint32_t      period       = s_cfg->health_period_ms;
    uint32_t            last_bus_off = 0;
    TickType_t          wake         = xTaskGetTickCount();
    lbr_health_record_t h;

    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(period));
        esp_task_wdt_reset();
        build_health(&h, period);

        if (h.can_bus_off_count != last_bus_off) {
            last_bus_off = h.can_bus_off_count;
            lbr_node_emit_event(LBR_EVT_CAN_BUS_OFF, (uint16_t)last_bus_off, true);
        }
        if (s_can_up) {
            for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
                lbr_can_frame_t f;
                lbr_can_encode_health_page(s_cfg->node_id, &h, p, &f);
                lbr_can_bus_send(&f);
            }
        }
        if (s_log != NULL) {
            lbr_log_append_health(s_log, &h);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Entry                                                                     */

void lbr_node_run(const lbr_node_config_t *cfg)
{
    s_cfg          = cfg;
    s_reset_reason = lbr_health_reset_reason();
    s_boot_count   = lbr_health_boot_count();
    s_evt_lock     = xSemaphoreCreateMutex();
    for (size_t i = 0; i < cfg->n_acq && i < LBR_NODE_MAX_SENSORS; i++) {
        s_sim = s_sim || cfg->acq[i].sensor->simulated;
    }

    lbr_node_config_text(cfg, s_config_text, sizeof s_config_text);
    s_config_crc = lbr_crc32(s_config_text, strlen(s_config_text));

    ESP_LOGI(TAG, "%s (0x%02x) fw %u.%u.%u protocol %u boot #%lu reset %d%s",
             lbr_node_name(cfg->node_id), cfg->node_id, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
             LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION, (unsigned long)s_boot_count,
             (int)s_reset_reason, s_sim ? " SIMULATED SENSORS" : "");

    lbr_can_bus_config_t can_cfg  = lbr_can_bus_default_config();
    char                 err[128] = "";
    s_config_valid =
        lbr_node_config_validate(cfg, can_cfg.bitrate, CONFIG_LBR_CAN_NODE_BUDGET_PERMILLE, err,
                                 sizeof err) == 0;
    if (!s_config_valid) {
        ESP_LOGE(TAG, "configuration rejected: %s", err);
    }

    can_cfg.accept_all = cfg->listen_all;
    lbr_can_err_t can  = lbr_can_bus_start(&can_cfg);
    s_can_up           = can == LBR_CAN_OK;

    if (cfg->local_log && !start_local_log()) {
        ESP_LOGE(TAG, "local log init failed (out of memory)");
    }

    lbr_node_emit_event(LBR_EVT_BOOT, (uint16_t)s_reset_reason, false);
    if (can == LBR_CAN_ERR_PINS_UNSET) {
        lbr_node_emit_event(LBR_EVT_CAN_PINS_UNSET, 0, true);
    }
    if (!s_config_valid) {
        lbr_node_emit_event(LBR_EVT_CONFIG_INVALID, 0, true);
    } else {
        for (size_t i = 0; i < cfg->n_acq; i++) {
            acq_ctx_t *ctx = &s_acq[i];
            ctx->acq       = &cfg->acq[i];
            ctx->index     = (uint8_t)i;
            atomic_store(&ctx->calibrated, true);
            esp_timer_create_args_t targs = {
                .callback = acq_timer_cb,
                .arg      = ctx,
                .name     = "lbr_acq",
            };
            char name[16];
            snprintf(name, sizeof name, "acq_%u", (unsigned)i);
            /* Timer first: the task starts it as soon as it runs. */
            if (esp_timer_create(&targs, &ctx->timer) != ESP_OK ||
                xTaskCreate(acq_task, name, 4096, ctx, ACQ_TASK_PRIO, &ctx->task) != pdPASS) {
                ESP_LOGE(TAG, "cannot start acquisition for %s", ctx->acq->sensor->name);
            }
        }
    }

    if (s_can_up) {
        xTaskCreate(rx_task, "lbr_rx", 4096, NULL, RX_TASK_PRIO, NULL);
    }
    xTaskCreate(health_task, "lbr_health", 4096, NULL, HEALTH_TASK_PRIO, NULL);

    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
