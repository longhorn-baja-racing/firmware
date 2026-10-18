#include "dashboard_plan.h"
#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/can.h"
#include "lbr/node_table.h"
#include "lbr/timebase.h"
#include "lbr/value_table.h"
#include "lbr/node_app.h"
#include "sdkconfig.h"

/*
 * Cockpit dashboard node: shows live values and node status to the driver,
 * and publishes its own buttons through the shared node runtime.
 */

#define RENDER_PERIOD_MS 500
#define STALE_US         2000000u
#define NODE_TIMEOUT_US  3000000u

static lbr_sensor_t      s_sensors[DASHBOARD_SLOT_COUNT];
static lbr_acq_t         s_acq[DASHBOARD_SLOT_COUNT];
static lbr_node_config_t s_cfg;

static SemaphoreHandle_t s_lock;
static lbr_value_table_t s_values;
static lbr_node_table_t  s_nodes;

static void on_frame(const lbr_can_frame_t *f, uint64_t rx_shared_us, void *ctx)
{
    (void)ctx;
    lbr_sample_t s;
    bool         is_sample = lbr_can_decode_sample(f, rx_shared_us, &s);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    lbr_node_table_on_frame(&s_nodes, f, rx_shared_us);
    if (is_sample) {
        lbr_value_table_offer(&s_values, &s, rx_shared_us);
    }
    xSemaphoreGive(s_lock);
}

static void display_task(void *arg)
{
    (void)arg;
    /* Render from a snapshot: drawing is slow and must not hold up CAN RX. */
    static lbr_value_table_t values;
    static lbr_node_table_t  nodes;
    uint8_t                  lost[LBR_NODE_TABLE_SIZE];
    dash_display_init();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RENDER_PERIOD_MS));
        uint64_t now = lbr_time_now_us();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        lbr_node_table_expire(&s_nodes, now, lost, sizeof lost);
        values = s_values;
        nodes  = s_nodes;
        xSemaphoreGive(s_lock);
        dash_display_render(&values, &nodes, now);
    }
}

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    s_lock = xSemaphoreCreateMutex();
    lbr_value_table_init(&s_values, STALE_US);
    lbr_node_table_init(&s_nodes, NODE_TIMEOUT_US);
    for (size_t i = 0; i < lbr_dashboard_watch_count; i++) {
        lbr_value_table_watch(&s_values, lbr_dashboard_watch[i], 0);
    }

    lbr_node_build(&lbr_plan_dashboard, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);
    /* Buttons on GPIOs are set in menuconfig (CONFIG_LBR_IO_BUTTONS); a panel
     * with its own driver replaces the slot instead (issue #10). */
    lbr_node_bind_io(&lbr_plan_dashboard, DASHBOARD_SLOT_BUTTONS, s_sensors, CONFIG_LBR_IO_BUTTONS);
    s_cfg.on_frame = on_frame;

    xTaskCreate(display_task, "dash_display", 4096, NULL, 4, NULL);
    lbr_node_run(&s_cfg);
}
