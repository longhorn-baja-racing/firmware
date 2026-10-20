#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu_plan.h"
#include "lbr/corner_rx.h"
#include "lbr/node_app.h"
#include "lbr/node_core.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/timebase.h"
#include "lbr/wlink.h"
#include "lbr/wlink_esp.h"
#include "sdkconfig.h"

/*
 * IMU Controller app (ESP32-C5, CAN node 1): the chassis IMU through the
 * shared node runtime, plus the corner network. It runs the access point the
 * four corner nodes join, answers their clock requests, and forwards every
 * sample they send onto CAN in its frame-table frames (lbr/corner_rx.h, run
 * unchanged by the simulator).
 */

#define SERVICE_TASK_PRIO 8

static lbr_sensor_t      s_sensors[IMU_SLOT_COUNT];
static lbr_acq_t         s_acq[IMU_SLOT_COUNT];
static lbr_node_config_t s_cfg;

static lbr_corner_rx_t   s_rx;
static SemaphoreHandle_t s_rx_lock;
static lbr_udp_t        *s_udp;
static uint32_t          s_corner_ip[LBR_CORNER_COUNT]; /* last address heard from */
static uint16_t          s_corner_port[LBR_CORNER_COUNT];

/* ------------------------------------------------------------------------ */
/* Corner receiver platform                                                  */

static uint64_t rx_now(void *ctx)
{
    (void)ctx;
    return lbr_time_now_us();
}

static bool rx_synced(void *ctx)
{
    (void)ctx;
    return lbr_time_is_synced();
}

static uint16_t rx_epoch(void *ctx)
{
    (void)ctx;
    lbr_time_servo_t ts;
    lbr_time_get_servo(&ts);
    return ts.epoch;
}

static bool rx_send_frame(void *ctx, const lbr_can_frame_t *f)
{
    (void)ctx;
    lbr_node_core_t *core = lbr_node_running_core();
    return core != NULL && lbr_node_core_send(core, f);
}

static bool rx_reply(void *ctx, uint8_t node, const uint8_t *pkt, size_t len)
{
    (void)ctx;
    int c = lbr_node_corner_index(node);
    return c >= 0 && lbr_udp_send(s_udp, s_corner_ip[c], s_corner_port[c], pkt, len);
}

static void rx_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    (void)ctx;
    lbr_node_emit_event(code, arg, fault);
}

static void rx_lock(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_rx_lock, portMAX_DELAY);
}

static void rx_unlock(void *ctx)
{
    (void)ctx;
    xSemaphoreGive(s_rx_lock);
}

/* ------------------------------------------------------------------------ */
/* Tasks                                                                     */

static void on_datagram(lbr_udp_t *self, const uint8_t *buf, size_t len, uint32_t ip, uint16_t port,
                        uint64_t rx_local_us, void *ctx)
{
    s_udp = self; /* the reply path may run before lbr_udp_open returned */
    (void)ctx;
    /* Remember where each corner is, for its clock replies. */
    if (len >= LBR_PKT_HEADER_SIZE) {
        int c = lbr_node_corner_index(buf[2]);
        if (c >= 0) {
            s_corner_ip[c]   = ip;
            s_corner_port[c] = port;
        }
    }
    lbr_corner_rx_on_packet(&s_rx, buf, len, lbr_time_local_to_shared(rx_local_us));
}

static void service_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LBR_CORNER_SERVICE_MS));
        lbr_node_core_t *core = lbr_node_running_core();
        if (core != NULL) {
            lbr_corner_rx_service(&s_rx, core);
        }
    }
}

/* Runs once the node core is up (lbr_node_run). */
static void start_corner_network(void *ctx)
{
    (void)ctx;
    if (!lbr_wifi_start_ap(CONFIG_LBR_CORNER_SSID, CONFIG_LBR_CORNER_PASSWORD,
                           CONFIG_LBR_CORNER_CHANNEL) ||
        (s_udp = lbr_udp_open(LBR_WLINK_CORNER_PORT, on_datagram, NULL)) == NULL) {
        /* No corner network: the corners read as down (LINK in the heartbeat). */
        lbr_node_emit_event(LBR_EVT_CONFIG_INVALID, 1, true);
    }
    xTaskCreate(service_task, "corner_svc", 4096, NULL, SERVICE_TASK_PRIO, NULL);
}

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    s_rx_lock = xSemaphoreCreateMutex();
    lbr_node_build(&lbr_plan_imu, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);

    /*
     * The chassis IMU waits for a part-specific driver (issue #10), bound
     * the same way:
     *
     *     lbr_sensor_<part>(&s_sensors[IMU_SLOT_CHASSIS], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */
    lbr_corner_rx_hal_t hal = {
        .now_us     = rx_now,
        .synced     = rx_synced,
        .epoch      = rx_epoch,
        .send_frame = rx_send_frame,
        .reply      = rx_reply,
        .event      = rx_event,
        .lock       = rx_lock,
        .unlock     = rx_unlock,
    };
    lbr_corner_rx_init(&s_rx, &hal);
    const lbr_slot_t *st = &lbr_plan_imu.slots[IMU_SLOT_CORNER_STATUS];
    lbr_sensor_corner_status(&s_sensors[IMU_SLOT_CORNER_STATUS], st->name, st->signals,
                             st->n_signals, &s_rx);

    s_cfg.on_started = start_corner_network;
    lbr_node_run(&s_cfg);
}
