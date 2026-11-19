#include "lbr/driver.h"

/* Pure adapter from lbr_driver to the runtime's lbr_sensor_t; see lbr/driver.h. */

static void set(lbr_channels *ch, size_t i, lbr_reading_t r)
{
    if (ch != NULL && i < ch->n) {
        ch->out[i] = r;
    }
}

void lbr_ch_f32(lbr_channels *ch, size_t i, float v)
{
    set(ch, i, (lbr_reading_t){.valid = true, .v.f32 = v});
}

void lbr_ch_i32(lbr_channels *ch, size_t i, int32_t v)
{
    set(ch, i, (lbr_reading_t){.valid = true, .v.i32 = v});
}

void lbr_ch_u32(lbr_channels *ch, size_t i, uint32_t v)
{
    set(ch, i, (lbr_reading_t){.valid = true, .v.u32 = v});
}

void lbr_ch_bool(lbr_channels *ch, size_t i, bool v)
{
    set(ch, i, (lbr_reading_t){.valid = true, .v.b = v});
}

void lbr_ch_missing(lbr_channels *ch, size_t i, lbr_unavail_reason_t why)
{
    set(ch, i, (lbr_reading_t){.valid = false, .reason = why});
}

void lbr_ch_missing_all(lbr_channels *ch, lbr_unavail_reason_t why)
{
    for (size_t i = 0; ch != NULL && i < ch->n; i++) {
        lbr_ch_missing(ch, i, why);
    }
}

static int driver_init(lbr_sensor_t *self)
{
    return self->driver->start != NULL ? self->driver->start(self->ctx) : 0;
}

static int driver_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    /* Anything the driver leaves unset is NO_DATA, never a stale or zero value. */
    lbr_channels ch = {.out = out, .signals = self->signals, .n = self->n_signals, .t_us = t_us};
    lbr_ch_missing_all(&ch, LBR_UNAVAIL_NO_DATA);
    return self->driver->read(self->ctx, &ch);
}

static bool driver_calibrated(lbr_sensor_t *self)
{
    return self->driver->calibrated(self->ctx);
}

bool lbr_sensor_use_driver(lbr_sensor_t *slot, const lbr_driver *drv, void *self)
{
    if (slot == NULL || drv == NULL || drv->read == NULL || drv->part == NULL ||
        drv->channels != slot->n_signals) {
        return false;
    }
    slot->init          = driver_init;
    slot->read          = driver_read;
    slot->is_calibrated = drv->calibrated != NULL ? driver_calibrated : NULL;
    slot->simulated     = false;
    slot->params        = drv->part; /* the session config records which part runs the slot */
    slot->driver        = drv;
    slot->ctx           = self;
    return true;
}
