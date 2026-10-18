#include "lbr/can_bus.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lbr/nodes.h"
#include "sdkconfig.h"

static const char *TAG = "lbr_can";

/*
 * The TWAI driver keeps a pointer to each queued frame until its TX-done
 * callback, so outgoing frames live in a fixed slot pool. Free slot indexes
 * travel through a FreeRTOS queue: taken by senders, returned from the ISR.
 */
#define MAX_TX_SLOTS 64

typedef struct {
    twai_frame_t frame;
    uint8_t      data[8];
} tx_slot_t;

static twai_node_handle_t s_node;
static QueueHandle_t      s_rx_q;
static QueueHandle_t      s_free_slots;
static tx_slot_t          s_slots[MAX_TX_SLOTS];
static size_t             s_slot_count;
static TaskHandle_t       s_recover_task;
static TaskHandle_t       s_sync_waiter;
static volatile uint64_t  s_sync_tx_us;
static uint32_t           s_sync_id;

static atomic_uint s_tx_frames;
static atomic_uint s_rx_frames;
static atomic_uint s_tx_dropped;
static atomic_uint s_tx_failed;
static atomic_uint s_rx_dropped;
static atomic_uint s_rx_foreign;
static atomic_uint s_bus_off_count;
static atomic_uint s_bus_errors;
static atomic_bool s_running;

lbr_can_bus_config_t lbr_can_bus_default_config(void)
{
    lbr_can_bus_config_t cfg = {
        .bitrate      = CONFIG_LBR_CAN_BITRATE,
        .tx_gpio      = CONFIG_LBR_CAN_TX_GPIO,
        .rx_gpio      = CONFIG_LBR_CAN_RX_GPIO,
        .rx_queue_len = CONFIG_LBR_CAN_RX_QUEUE_LEN,
        .tx_queue_len = CONFIG_LBR_CAN_TX_QUEUE_LEN,
        .accept_all   = true,
    };
    return cfg;
}

static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *ed,
                                 void *ctx)
{
    (void)ed;
    (void)ctx;
    lbr_can_rx_t rx  = {.rx_local_us = (uint64_t)esp_timer_get_time()};
    twai_frame_t frm = {.buffer = rx.frame.data, .buffer_len = sizeof rx.frame.data};
    if (twai_node_receive_from_isr(handle, &frm) != ESP_OK) {
        return false;
    }
    if (!frm.header.ide || frm.header.rtr) {
        /* Not an LBR frame (LBR uses extended data frames only); counted. */
        atomic_fetch_add(&s_rx_foreign, 1);
        return false;
    }
    rx.frame.id  = frm.header.id;
    rx.frame.dlc = (uint8_t)(frm.header.dlc > 8 ? 8 : frm.header.dlc);

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_q, &rx, &woken) == pdTRUE) {
        atomic_fetch_add(&s_rx_frames, 1);
    } else {
        atomic_fetch_add(&s_rx_dropped, 1);
    }
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *ed,
                                 void *ctx)
{
    (void)handle;
    (void)ctx;
    BaseType_t woken = pdFALSE;
    if (ed->is_tx_success) {
        atomic_fetch_add(&s_tx_frames, 1);
    } else {
        atomic_fetch_add(&s_tx_failed, 1);
    }
    const twai_frame_t *done = ed->done_tx_frame;
    if (done != NULL && done->header.id == s_sync_id && ed->is_tx_success) {
        s_sync_tx_us = (uint64_t)esp_timer_get_time();
        if (s_sync_waiter != NULL) {
            vTaskNotifyGiveFromISR(s_sync_waiter, &woken);
        }
    }
    for (size_t i = 0; i < s_slot_count; i++) {
        if (done == &s_slots[i].frame) {
            uint8_t idx = (uint8_t)i;
            xQueueSendFromISR(s_free_slots, &idx, &woken);
            break;
        }
    }
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t                    handle,
                                      const twai_state_change_event_data_t *ed, void *ctx)
{
    (void)handle;
    (void)ctx;
    BaseType_t woken = pdFALSE;
    if (ed->new_sta == TWAI_ERROR_BUS_OFF) {
        atomic_fetch_add(&s_bus_off_count, 1);
        atomic_store(&s_running, false);
        vTaskNotifyGiveFromISR(s_recover_task, &woken);
    } else if (ed->old_sta == TWAI_ERROR_BUS_OFF && ed->new_sta == TWAI_ERROR_ACTIVE) {
        atomic_store(&s_running, true);
    }
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_error(twai_node_handle_t handle, const twai_error_event_data_t *ed,
                               void *ctx)
{
    (void)handle;
    (void)ed;
    (void)ctx;
    atomic_fetch_add(&s_bus_errors, 1);
    return false;
}

static void recover_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Back off briefly so a shorted bus does not spin the controller. */
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGW(TAG, "bus-off: starting recovery");
        if (twai_node_recover(s_node) != ESP_OK) {
            ESP_LOGE(TAG, "bus-off recovery request failed");
        }
    }
}

lbr_can_err_t lbr_can_bus_start(const lbr_can_bus_config_t *cfg)
{
    if (cfg->tx_gpio < 0 || cfg->rx_gpio < 0) {
        ESP_LOGE(TAG, "CAN GPIOs not configured (CONFIG_LBR_CAN_TX_GPIO/RX_GPIO are TBD)");
        return LBR_CAN_ERR_PINS_UNSET;
    }

    s_slot_count = cfg->tx_queue_len + 1;
    if (s_slot_count > MAX_TX_SLOTS) {
        s_slot_count = MAX_TX_SLOTS;
    }
    s_rx_q       = xQueueCreate(cfg->rx_queue_len, sizeof(lbr_can_rx_t));
    s_free_slots = xQueueCreate(s_slot_count, sizeof(uint8_t));
    if (s_rx_q == NULL || s_free_slots == NULL) {
        return LBR_CAN_ERR_NO_MEM;
    }
    for (size_t i = 0; i < s_slot_count; i++) {
        uint8_t idx = (uint8_t)i;
        xQueueSend(s_free_slots, &idx, 0);
    }
    s_sync_id = lbr_can_id_make(LBR_CAN_PRIO_TIME, LBR_CAN_KIND_SYSTEM, LBR_NODE_GATEWAY,
                                LBR_SYS_TIME_SYNC);

    if (xTaskCreate(recover_task, "lbr_can_rcv", 3072, NULL, 10, &s_recover_task) != pdPASS) {
        return LBR_CAN_ERR_NO_MEM;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg =
            {
                .tx                = (gpio_num_t)cfg->tx_gpio,
                .rx                = (gpio_num_t)cfg->rx_gpio,
                .quanta_clk_out    = GPIO_NUM_NC,
                .bus_off_indicator = GPIO_NUM_NC,
            },
        .bit_timing     = {.bitrate = cfg->bitrate},
        .fail_retry_cnt = -1,
        .tx_queue_depth = cfg->tx_queue_len,
        .intr_priority  = 0,
    };
    if (twai_new_node_onchip(&node_cfg, &s_node) != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed");
        return LBR_CAN_ERR_DRIVER;
    }

    if (!cfg->accept_all) {
        /* Only system frames from the gateway (time sync). */
        twai_mask_filter_config_t filt = {
            .id     = ((uint32_t)LBR_CAN_KIND_SYSTEM << 24) | ((uint32_t)LBR_NODE_GATEWAY << 16),
            .mask   = (0x3u << 24) | (0xFFu << 16),
            .is_ext = true,
        };
        if (twai_node_config_mask_filter(s_node, 0, &filt) != ESP_OK) {
            ESP_LOGE(TAG, "filter config failed");
            return LBR_CAN_ERR_DRIVER;
        }
    }

    twai_event_callbacks_t cbs = {
        .on_tx_done      = on_tx_done,
        .on_rx_done      = on_rx_done,
        .on_state_change = on_state_change,
        .on_error        = on_error,
    };
    if (twai_node_register_event_callbacks(s_node, &cbs, NULL) != ESP_OK ||
        twai_node_enable(s_node) != ESP_OK) {
        ESP_LOGE(TAG, "TWAI enable failed");
        return LBR_CAN_ERR_DRIVER;
    }
    atomic_store(&s_running, true);
    ESP_LOGI(TAG, "TWAI up: %lu bit/s, tx=%d rx=%d", (unsigned long)cfg->bitrate, cfg->tx_gpio,
             cfg->rx_gpio);
    return LBR_CAN_OK;
}

bool lbr_can_bus_send(const lbr_can_frame_t *f)
{
    uint8_t idx;
    if (s_node == NULL || !atomic_load(&s_running) || f->dlc > 8 ||
        xQueueReceive(s_free_slots, &idx, 0) != pdTRUE) {
        atomic_fetch_add(&s_tx_dropped, 1);
        return false;
    }
    tx_slot_t *slot = &s_slots[idx];
    memcpy(slot->data, f->data, f->dlc);
    slot->frame = (twai_frame_t){
        .header     = {.id = f->id, .ide = 1, .dlc = f->dlc},
        .buffer     = slot->data,
        .buffer_len = f->dlc,
    };
    if (twai_node_transmit(s_node, &slot->frame, 0) != ESP_OK) {
        xQueueSend(s_free_slots, &idx, 0);
        atomic_fetch_add(&s_tx_dropped, 1);
        return false;
    }
    return true;
}

bool lbr_can_bus_receive(lbr_can_rx_t *out, uint32_t timeout_ms)
{
    if (s_rx_q == NULL) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return false;
    }
    return xQueueReceive(s_rx_q, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void lbr_can_bus_arm_sync_tx(void)
{
    s_sync_waiter = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);
}

bool lbr_can_bus_wait_sync_tx_done(uint64_t *tx_local_us, uint32_t timeout_ms)
{
    bool got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
    if (got) {
        *tx_local_us = s_sync_tx_us;
    }
    return got;
}

void lbr_can_bus_get_stats(lbr_can_stats_t *out)
{
    memset(out, 0, sizeof *out);
    out->tx_frames     = atomic_load(&s_tx_frames);
    out->rx_frames     = atomic_load(&s_rx_frames);
    out->tx_dropped    = atomic_load(&s_tx_dropped);
    out->tx_failed     = atomic_load(&s_tx_failed);
    out->rx_dropped    = atomic_load(&s_rx_dropped);
    out->rx_foreign    = atomic_load(&s_rx_foreign);
    out->bus_off_count = atomic_load(&s_bus_off_count);
    out->bus_errors    = atomic_load(&s_bus_errors);
    out->running       = atomic_load(&s_running);
    if (s_node != NULL) {
        twai_node_status_t st;
        if (twai_node_get_info(s_node, &st, NULL) == ESP_OK) {
            out->tec = (uint8_t)(st.tx_error_count > 255 ? 255 : st.tx_error_count);
            out->rec = (uint8_t)(st.rx_error_count > 255 ? 255 : st.rx_error_count);
        }
    }
}
