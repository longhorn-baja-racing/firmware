#include "lbr/log_service.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/protocol.h"
#include "lbr/version.h"

static const char *TAG = "lbr_logsvc";

#define PUMP_PERIOD_MS 20
#define MOUNT_RETRY_US 5000000u
#define CHUNK_SIZE     8192
#define TASK_PRIO      5

static lbr_log_service_config_t s_cfg;
static lbr_logger_t             s_log;
static SemaphoreHandle_t        s_lock;
static atomic_bool              s_sd_present;
static char                     s_name[LBR_SD_NAME_MAX];
static portMUX_TYPE             s_name_mux = portMUX_INITIALIZER_UNLOCKED;

static void lock(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void *ctx)
{
    (void)ctx;
    xSemaphoreGive(s_lock);
}

static void set_name(const char *name)
{
    portENTER_CRITICAL(&s_name_mux);
    strncpy(s_name, name, sizeof s_name - 1);
    s_name[sizeof s_name - 1] = '\0';
    portEXIT_CRITICAL(&s_name_mux);
}

static void event(uint16_t code, bool fault)
{
    if (s_cfg.on_event != NULL) {
        s_cfg.on_event(code, fault);
    }
}

static bool open_session(void)
{
    lbr_log_storage_t st;
    char              name[LBR_SD_NAME_MAX];
    lbr_log_session_t session = {
        .fw_version    = LBR_FW_VERSION,
        .node_id       = s_cfg.node_id,
        .reset_reason  = s_cfg.reset_reason,
        .boot_count    = s_cfg.boot_count,
        .start_time_us = s_cfg.now_us(),
        .config_text   = s_cfg.config_text,
    };
    if (lbr_sd_open_session(&st, name) != LBR_SD_OK) {
        return false;
    }
    if (lbr_log_open(&s_log, &st, &session) != 0) {
        st.close(st.ctx);
        return false;
    }
    set_name(name);
    return true;
}

static void writer_task(void *arg)
{
    (void)arg;
    uint64_t next_mount  = 0;
    uint32_t last_errors = 0;
    bool     reported    = false;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PUMP_PERIOD_MS));
        uint64_t mono = (uint64_t)esp_timer_get_time();

        if (!lbr_log_is_open(&s_log)) {
            if (mono < next_mount) {
                continue;
            }
            next_mount = mono + MOUNT_RETRY_US;
            if (lbr_sd_mount() != LBR_SD_OK || !open_session()) {
                lbr_sd_unmount();
                atomic_store(&s_sd_present, false);
                if (!reported) {
                    event(LBR_EVT_SD_MISSING, true);
                    reported = true;
                }
                continue;
            }
            atomic_store(&s_sd_present, true);
            reported = false;
            lbr_log_stats_t ls;
            lbr_log_get_stats(&s_log, &ls);
            last_errors = ls.write_errors;
            event(LBR_EVT_SD_MOUNTED, false);
            continue;
        }

        lbr_log_pump(&s_log, s_cfg.now_us());
        lbr_log_stats_t ls;
        lbr_log_get_stats(&s_log, &ls);
        if (ls.write_errors != last_errors) {
            /* Card pulled or failing: close out, remount on a later pass. */
            ESP_LOGE(TAG, "SD write error; closing session and remounting");
            event(LBR_EVT_SD_WRITE_ERROR, true);
            lbr_log_close(&s_log, s_cfg.now_us());
            lbr_sd_unmount();
            set_name("");
            atomic_store(&s_sd_present, false);
            next_mount = mono + MOUNT_RETRY_US;
        }
    }
}

bool lbr_log_service_start(const lbr_log_service_config_t *cfg)
{
    s_cfg          = *cfg;
    s_lock         = xSemaphoreCreateMutex();
    uint8_t *queue = heap_caps_malloc(cfg->queue_size, MALLOC_CAP_8BIT);
    uint8_t *chunk = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    if (s_lock == NULL || queue == NULL || chunk == NULL) {
        ESP_LOGE(TAG, "out of memory for a %u byte log queue", (unsigned)cfg->queue_size);
        return false;
    }
    lbr_log_config_t lc = {
        .queue_mem        = queue,
        .queue_cap        = cfg->queue_size,
        .chunk            = chunk,
        .chunk_cap        = CHUNK_SIZE,
        .sync_interval_ms = cfg->sync_interval_ms,
        .lock             = lock,
        .unlock           = unlock,
    };
    if (lbr_log_init(&s_log, &lc) != 0) {
        return false;
    }
    return xTaskCreate(writer_task, "lbr_logw", 4096, NULL, TASK_PRIO, NULL) == pdPASS;
}

lbr_logger_t *lbr_log_service_logger(void)
{
    return &s_log;
}

bool lbr_log_service_sd_present(void)
{
    return atomic_load(&s_sd_present);
}

void lbr_log_service_file_name(char out[LBR_SD_NAME_MAX])
{
    portENTER_CRITICAL(&s_name_mux);
    memcpy(out, s_name, LBR_SD_NAME_MAX);
    portEXIT_CRITICAL(&s_name_mux);
}
