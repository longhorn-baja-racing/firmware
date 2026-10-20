#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lbr/sensor_io.h"
#include "sdkconfig.h"

/*
 * Hardware side of the part-independent inputs (see lbr/sensor_io.h).
 *
 * ADC1 is shared by every slot on the node, so the unit and its calibration
 * are created once, under a mutex made at construction time (app_main, before
 * any acquisition task runs). Pulse inputs count edges in an IRAM GPIO ISR,
 * so counting continues while flash is busy.
 */

#if CONFIG_IDF_TARGET_ESP32C5
#define LBR_IO_CHIP lbr_io_chip_esp32c5
#else
#define LBR_IO_CHIP lbr_io_chip_esp32
#endif

static const char *TAG = "lbr_io";

typedef struct {
    volatile uint32_t count;
    volatile uint64_t edge_us;
} pulse_cap_t;

typedef struct {
    lbr_io_chan_t  chan[LBR_SENSOR_MAX_SIGNALS];
    adc_channel_t  adc_ch[LBR_SENSOR_MAX_SIGNALS];
    pulse_cap_t    cap[LBR_SENSOR_MAX_SIGNALS];
    lbr_io_pulse_t pulse[LBR_SENSOR_MAX_SIGNALS];
    lbr_io_din_t   din[LBR_SENSOR_MAX_SIGNALS];
    bool           parsed;
    bool           ready;
} io_ctx_t;

static SemaphoreHandle_t         s_adc_lock;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static portMUX_TYPE              s_cap_lock = portMUX_INITIALIZER_UNLOCKED;
static bool                      s_isr_service;

static void IRAM_ATTR pulse_isr(void *arg)
{
    pulse_cap_t *cap = arg;
    uint64_t     now = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_cap_lock);
    cap->count   = cap->count + 1u;
    cap->edge_us = now;
    portEXIT_CRITICAL_ISR(&s_cap_lock);
}

static int adc_unit_up(void)
{
    if (s_adc != NULL) {
        return 0;
    }
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) {
        s_adc = NULL;
        return -1;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t ccfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cal = adc_cali_create_scheme_curve_fitting(&ccfg, &s_cali);
#else
    adc_cali_line_fitting_config_t ccfg = {
        .unit_id      = ADC_UNIT_1,
        .atten        = ADC_ATTEN_DB_12,
        .bitwidth     = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    esp_err_t cal = adc_cali_create_scheme_line_fitting(&ccfg, &s_cali);
#endif
    if (cal != ESP_OK) {
        /* Without calibration the pin voltage is not trustworthy. */
        s_cali = NULL;
        return -1;
    }
    return 0;
}

static gpio_pull_mode_t pull_mode(lbr_io_pull_t p)
{
    switch (p) {
        case LBR_IO_PULL_UP:
            return GPIO_PULLUP_ONLY;
        case LBR_IO_PULL_DOWN:
            return GPIO_PULLDOWN_ONLY;
        default:
            return GPIO_FLOATING;
    }
}

static int input_pin(int8_t gpio, lbr_io_pull_t pull, gpio_int_type_t intr)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ull << (unsigned)gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = intr,
    };
    if (gpio_config(&io) != ESP_OK) {
        return -1;
    }
    return gpio_set_pull_mode((gpio_num_t)gpio, pull_mode(pull)) == ESP_OK ? 0 : -1;
}

static int init_chan(io_ctx_t *ctx, size_t i)
{
    lbr_io_chan_t *c = &ctx->chan[i];
    switch (c->kind) {
        case LBR_IO_ADC: {
            adc_unit_t unit;
            if (adc_oneshot_io_to_channel(c->u.adc.gpio, &unit, &ctx->adc_ch[i]) != ESP_OK ||
                unit != ADC_UNIT_1) {
                return -1;
            }
            adc_oneshot_chan_cfg_t cc = {
                .atten    = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            xSemaphoreTake(s_adc_lock, portMAX_DELAY);
            int rc = adc_unit_up();
            if (rc == 0 && adc_oneshot_config_channel(s_adc, ctx->adc_ch[i], &cc) != ESP_OK) {
                rc = -1;
            }
            xSemaphoreGive(s_adc_lock);
            return rc;
        }
        case LBR_IO_PULSE: {
            if (input_pin(c->u.pulse.gpio, c->u.pulse.pull,
                          c->u.pulse.falling ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE) != 0) {
                return -1;
            }
            xSemaphoreTake(s_adc_lock, portMAX_DELAY);
            if (!s_isr_service) {
                esp_err_t e   = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
                s_isr_service = e == ESP_OK || e == ESP_ERR_INVALID_STATE;
            }
            bool ok = s_isr_service;
            xSemaphoreGive(s_adc_lock);
            if (!ok) {
                return -1;
            }
            gpio_isr_handler_remove((gpio_num_t)c->u.pulse.gpio);
            if (gpio_isr_handler_add((gpio_num_t)c->u.pulse.gpio, pulse_isr, &ctx->cap[i]) !=
                ESP_OK) {
                return -1;
            }
            portENTER_CRITICAL(&s_cap_lock);
            ctx->pulse[i] = (lbr_io_pulse_t){.ref_count = ctx->cap[i].count};
            portEXIT_CRITICAL(&s_cap_lock);
            return 0;
        }
        case LBR_IO_DIN:
            for (uint8_t k = 0; k < c->u.din.n_gpio; k++) {
                if (input_pin(c->u.din.gpio[k], c->u.din.pull, GPIO_INTR_DISABLE) != 0) {
                    return -1;
                }
            }
            ctx->din[i] = (lbr_io_din_t){0};
            return 0;
        default:
            return 0;
    }
}

static int io_init(lbr_sensor_t *self)
{
    io_ctx_t *ctx = self->ctx;
    if (ctx == NULL || !ctx->parsed) {
        return -1;
    }
    if (ctx->ready) {
        return 0;
    }
    for (size_t i = 0; i < self->n_signals; i++) {
        if (init_chan(ctx, i) != 0) {
            ESP_LOGE(TAG, "%s: channel %u init failed", self->name, (unsigned)i);
            return -1;
        }
    }
    ctx->ready = true;
    return 0;
}

static void read_adc(const lbr_io_chan_t *c, adc_channel_t ch, lbr_reading_t *out)
{
    int32_t sum = 0;
    for (uint8_t k = 0; k < c->u.adc.avg; k++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, ch, &raw) != ESP_OK) {
            *out = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_BUS_ERROR};
            return;
        }
        sum += raw;
    }
    int mv = 0;
    if (adc_cali_raw_to_voltage(s_cali, (int)(sum / c->u.adc.avg), &mv) != ESP_OK) {
        *out = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NOT_CALIBRATED};
        return;
    }
    lbr_io_adc_convert(c, (float)mv, out);
}

static int io_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us; /* pulse timing and debounce use the local clock: only differences matter */
    io_ctx_t *ctx = self->ctx;
    uint64_t  now = (uint64_t)esp_timer_get_time();
    for (size_t i = 0; i < self->n_signals; i++) {
        const lbr_io_chan_t *c = &ctx->chan[i];
        lbr_reading_t       *r = &out[i];
        *r                     = (lbr_reading_t){.valid = true};
        switch (c->kind) {
            case LBR_IO_ADC:
                read_adc(c, ctx->adc_ch[i], r);
                break;
            case LBR_IO_PULSE: {
                portENTER_CRITICAL(&s_cap_lock);
                uint32_t count = ctx->cap[i].count;
                uint64_t edge  = ctx->cap[i].edge_us;
                portEXIT_CRITICAL(&s_cap_lock);
                float hz =
                    lbr_io_pulse_update(&ctx->pulse[i], count, edge, now, c->u.pulse.stall_ms);
                r->v.f32 = hz * c->u.pulse.scale;
                break;
            }
            case LBR_IO_DIN: {
                uint32_t raw = 0;
                for (uint8_t k = 0; k < c->u.din.n_gpio; k++) {
                    bool level = gpio_get_level((gpio_num_t)c->u.din.gpio[k]) != 0;
                    if (level == c->u.din.active_high) {
                        raw |= 1u << k;
                    }
                }
                uint32_t v = lbr_io_din_update(&ctx->din[i], raw, now, c->u.din.debounce_ms);
                if (c->u.din.n_gpio == 1 &&
                    lbr_signal_info(self->signals[i])->type == LBR_VAL_BOOL) {
                    r->v.b = v != 0;
                } else {
                    r->v.u32 = v;
                }
                break;
            }
            default:
                *r = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NOT_FITTED};
                break;
        }
    }
    return 0;
}

static bool io_is_calibrated(lbr_sensor_t *self)
{
    const io_ctx_t *ctx = self->ctx;
    return ctx != NULL && ctx->parsed;
}

void lbr_sensor_io(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                   size_t n_signals, const char *spec)
{
    if (s_adc_lock == NULL) {
        s_adc_lock = xSemaphoreCreateMutex();
    }
    io_ctx_t *ctx = calloc(1, sizeof *ctx);
    if (ctx != NULL && n_signals <= LBR_SENSOR_MAX_SIGNALS && s_adc_lock != NULL) {
        char err[96];
        ctx->parsed =
            lbr_io_parse(spec, signals, n_signals, ctx->chan, &LBR_IO_CHIP, err, sizeof err) == 0;
        if (!ctx->parsed) {
            ESP_LOGE(TAG, "%s: bad input spec: %s", name, err);
        }
    }
    *out = (lbr_sensor_t){
        .name          = name,
        .signals       = signals,
        .n_signals     = n_signals,
        .init          = io_init,
        .read          = io_read,
        .is_calibrated = io_is_calibrated,
        .simulated     = false,
        .params        = spec,
        .ctx           = ctx,
    };
}
