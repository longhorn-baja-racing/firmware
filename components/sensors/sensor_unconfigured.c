#include "lbr/sensor.h"

static int unconfigured_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}

static int unconfigured_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us;
    for (size_t i = 0; i < self->n_signals; i++) {
        out[i].valid  = false;
        out[i].reason = LBR_UNAVAIL_NOT_FITTED;
        out[i].v.u32  = 0;
    }
    return 0;
}

void lbr_sensor_unconfigured(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                             size_t n_signals)
{
    *out = (lbr_sensor_t){
        .name          = name,
        .signals       = signals,
        .n_signals     = n_signals,
        .init          = unconfigured_init,
        .read          = unconfigured_read,
        .is_calibrated = NULL,
        .simulated     = false,
        .ctx           = NULL,
    };
}
