#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lbr/board.h"

/*
 * Time-sync probe (CONFIG_LBR_SYNC_PROBE_GPIO): a pulse at every whole second
 * of the shared clock, so a scope on two boards shows their sync error. The
 * task sleeps until about 2 ms before the edge, then spins on the shared
 * clock, so the edge's own jitter is a few microseconds, not an RTOS tick.
 */

static const char *TAG = "lbr_probe";

#define PULSE_US    100u
#define SPIN_MS     2u
#define SECOND_US   1000000u
#define PROBE_PRIO  (configMAX_PRIORITIES - 2)
#define PROBE_STACK 2048

static void probe_task(void *arg)
{
    gpio_num_t pin = (gpio_num_t)(intptr_t)arg;
    for (;;) {
        if (!lbr_time_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        uint64_t now  = lbr_time_now_us();
        uint64_t edge = (now / SECOND_US + 1u) * SECOND_US;
        uint32_t ms   = (uint32_t)((edge - now) / 1000u);
        if (ms > SPIN_MS) {
            vTaskDelay(pdMS_TO_TICKS(ms - SPIN_MS));
        }
        while (lbr_time_now_us() < edge) {
            /* spin: the edge is less than SPIN_MS away */
        }
        gpio_set_level(pin, 1);
        esp_rom_delay_us(PULSE_US);
        gpio_set_level(pin, 0);
    }
}

void lbr_sync_probe_start(void)
{
#if CONFIG_LBR_SYNC_PROBE_GPIO >= 0
    gpio_num_t pin = (gpio_num_t)CONFIG_LBR_SYNC_PROBE_GPIO;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        ESP_LOGE(TAG, "sync probe: GPIO %d cannot drive an output", (int)pin);
        return;
    }
    gpio_config_t io = {.pin_bit_mask = 1ull << pin, .mode = GPIO_MODE_OUTPUT};
    if (gpio_config(&io) != ESP_OK || gpio_set_level(pin, 0) != ESP_OK) {
        ESP_LOGE(TAG, "sync probe: GPIO %d setup failed", (int)pin);
        return;
    }
    if (xTaskCreate(probe_task, "lbr_probe", PROBE_STACK, (void *)(intptr_t)pin, PROBE_PRIO,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "sync probe: task not started");
        return;
    }
    ESP_LOGW(TAG, "sync probe on GPIO %d (bench only: a pulse every shared second)", (int)pin);
#endif
}
