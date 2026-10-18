#include <stdlib.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lbr/nmea.h"

/*
 * UART side of the NMEA GPS driver (see lbr/nmea.h). read() drains whatever
 * the UART driver has buffered without waiting, so it never blocks the
 * acquisition task; the driver's RX buffer holds several seconds at 9600 baud.
 */

static const char *TAG = "lbr_gps";

#define RX_BUF   2048
#define READ_BUF 256

typedef struct {
    lbr_gps_uart_cfg_t cfg;
    lbr_nmea_t         nmea;
    bool               installed;
} gps_ctx_t;

static int gps_init(lbr_sensor_t *self)
{
    gps_ctx_t *ctx = self->ctx;
    if (ctx == NULL || ctx->cfg.rx_gpio < 0) {
        return -1;
    }
    if (ctx->installed) {
        return 0;
    }
    uart_config_t ucfg = {
        .baud_rate  = (int)ctx->cfg.baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_port_t port = (uart_port_t)ctx->cfg.uart_num;
    if (uart_driver_install(port, RX_BUF, 0, 0, NULL, 0) != ESP_OK) {
        return -1;
    }
    if (uart_param_config(port, &ucfg) != ESP_OK ||
        uart_set_pin(port, ctx->cfg.tx_gpio < 0 ? UART_PIN_NO_CHANGE : ctx->cfg.tx_gpio,
                     ctx->cfg.rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        uart_driver_delete(port);
        return -1;
    }
    lbr_nmea_init(&ctx->nmea);
    ctx->installed = true;
    return 0;
}

static int gps_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us; /* staleness uses the local clock; samples are stamped by the runtime */
    gps_ctx_t *ctx = self->ctx;
    uint8_t    buf[READ_BUF];
    uint64_t   now = (uint64_t)esp_timer_get_time();
    for (int guard = 0; guard < RX_BUF / READ_BUF; guard++) {
        int n = uart_read_bytes((uart_port_t)ctx->cfg.uart_num, buf, sizeof buf, 0);
        if (n <= 0) {
            break;
        }
        lbr_nmea_feed(&ctx->nmea, buf, (size_t)n, now);
    }
    lbr_nmea_fill(&ctx->nmea, now, self->signals, self->n_signals, out);
    return 0;
}

void lbr_sensor_gps_nmea(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                         size_t n_signals, const lbr_gps_uart_cfg_t *cfg)
{
    gps_ctx_t *ctx = calloc(1, sizeof *ctx);
    if (ctx != NULL) {
        ctx->cfg = *cfg;
    } else {
        ESP_LOGE(TAG, "%s: out of memory", name);
    }
    *out = (lbr_sensor_t){
        .name          = name,
        .signals       = signals,
        .n_signals     = n_signals,
        .init          = gps_init,
        .read          = gps_read,
        .is_calibrated = NULL,
        .simulated     = false,
        .params        = NULL,
        .ctx           = ctx,
    };
}
