#include <math.h>

#include "lbr/sensor.h"

#define TWO_PI 6.28318530718f

static int sim_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}

/* Amplitude/offset roughly in the range a real channel of that unit shows,
 * so dashboards and plots behave sensibly during bench tests. */
static void unit_shape(lbr_unit_t unit, float *offset, float *amp)
{
    switch (unit) {
        case LBR_UNIT_DEG_C:
            *offset = 80.0f;
            *amp    = 20.0f;
            break;
        case LBR_UNIT_PERCENT:
            *offset = 50.0f;
            *amp    = 45.0f;
            break;
        case LBR_UNIT_M_PER_S2:
            *offset = 0.0f;
            *amp    = 9.8f;
            break;
        case LBR_UNIT_KPA:
            *offset = 1500.0f;
            *amp    = 1400.0f;
            break;
        case LBR_UNIT_MM:
            *offset = 0.0f;
            *amp    = 120.0f;
            break;
        case LBR_UNIT_MICROSTRAIN:
            *offset = 0.0f;
            *amp    = 800.0f;
            break;
        case LBR_UNIT_RPM:
            *offset = 2500.0f;
            *amp    = 1200.0f;
            break;
        case LBR_UNIT_VOLT:
            *offset = 12.6f;
            *amp    = 0.4f;
            break;
        default:
            *offset = 0.0f;
            *amp    = 1.0f;
            break;
    }
}

static int sim_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    /* Wrap time to keep float precision (period-aligned: 0.5 Hz base). */
    float t = (float)(t_us % 120000000u) / 1e6f;
    for (size_t i = 0; i < self->n_signals; i++) {
        const lbr_signal_info_t *info  = lbr_signal_info(self->signals[i]);
        float                    phase = (float)i * 0.7f;
        float                    s     = sinf(TWO_PI * 0.5f * t + phase);
        lbr_reading_t           *r     = &out[i];
        r->valid                       = info != NULL;
        r->reason                      = LBR_UNAVAIL_UNSPECIFIED;
        r->v.u32                       = 0;
        if (info == NULL) {
            r->reason = LBR_UNAVAIL_NOT_FITTED;
            continue;
        }
        switch (info->type) {
            case LBR_VAL_F32: {
                float offset, amp;
                unit_shape(info->unit, &offset, &amp);
                r->v.f32 = offset + amp * s;
                break;
            }
            case LBR_VAL_I32:
                r->v.i32 = (int32_t)(1000.0f * s);
                break;
            case LBR_VAL_U32:
                r->v.u32 = (uint32_t)(t_us / 1000000u);
                break;
            case LBR_VAL_BOOL:
                r->v.b = s > 0.0f;
                break;
            default:
                r->v.u32 = 0;
                break;
        }
    }
    return 0;
}

void lbr_sensor_sim(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                    size_t n_signals)
{
    *out = (lbr_sensor_t){
        .name          = name,
        .signals       = signals,
        .n_signals     = n_signals,
        .init          = sim_init,
        .read          = sim_read,
        .is_calibrated = NULL,
        .simulated     = true,
        .ctx           = NULL,
    };
}
