#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/sensor.h"

/*
 * Writing a part driver.
 *
 * A driver is two functions and a description. It never deals with time
 * sync, sequence numbers, CAN, or logging, and it allocates nothing: its
 * state is a struct the app owns.
 *
 *     typedef struct { int adc_pin; float volts_per_bar; } pressure_t;
 *
 *     static int pressure_start(void *self)
 *     {
 *         pressure_t *p = self;
 *         return adc_setup(p->adc_pin);           // 0 = up; <0 = not responding
 *     }
 *
 *     static int pressure_read(void *self, lbr_channels *ch)
 *     {
 *         pressure_t *p = self;
 *         int mv;
 *         if (adc_read_mv(p->adc_pin, &mv) != 0) {
 *             lbr_ch_missing_all(ch, LBR_UNAVAIL_BUS_ERROR);
 *             return 0;                            // a bad read, not a dead part
 *         }
 *         lbr_ch_f32(ch, 0, mv / 1000.0f / p->volts_per_bar);
 *         return 0;
 *     }
 *
 *     const lbr_driver pressure_driver = {
 *         .part = "pressure", .channels = 1,
 *         .start = pressure_start, .read = pressure_read,
 *     };
 *
 * and in the node's init():
 *
 *     static pressure_t brake_line = {.adc_pin = 34, .volts_per_bar = 0.05f};
 *     lbr_bind(n, BRAKES_SLOT_LINE, &pressure_driver, &brake_line);
 *
 * Channels are the slot's signals in catalog order. Any channel read() does
 * not set reports NO_DATA, so forgetting one is visible, never a zero.
 * If start() fails, every channel reports SENSOR_FAULT, a SENSOR_INIT_FAIL
 * fault goes on the bus, and the runtime retries start() every 2 s; read()
 * returning <0 means the part stopped answering and gets the same treatment.
 * read() must finish well inside one sample period.
 */

typedef struct lbr_channels lbr_channels;

typedef struct lbr_driver {
    const char *part;     /* part name, recorded in the session config */
    size_t      channels; /* how many channels it produces: must equal its slot's */

    int (*start)(void *self);                  /* bring the part up: 0, or <0 */
    int (*read)(void *self, lbr_channels *ch); /* one sample of every channel: 0, or <0 */
    bool (*calibrated)(void *self);            /* optional: false while uncalibrated */
} lbr_driver;

/* The channels of one read(). */
struct lbr_channels {
    lbr_reading_t         *out;
    const lbr_signal_id_t *signals;
    size_t                 n;
    uint64_t               t_us; /* sample time on the shared clock */
};

/* Set channel i (out-of-range indexes are ignored). */
void lbr_ch_f32(lbr_channels *ch, size_t i, float v);
void lbr_ch_i32(lbr_channels *ch, size_t i, int32_t v);
void lbr_ch_u32(lbr_channels *ch, size_t i, uint32_t v);
void lbr_ch_bool(lbr_channels *ch, size_t i, bool v);

/* Channel i (or every channel) has no value this time, and why. */
void lbr_ch_missing(lbr_channels *ch, size_t i, lbr_unavail_reason_t why);
void lbr_ch_missing_all(lbr_channels *ch, lbr_unavail_reason_t why);

/* Number of channels, and the sample time on the shared clock. */
static inline size_t lbr_ch_count(const lbr_channels *ch)
{
    return ch->n;
}
static inline uint64_t lbr_ch_time_us(const lbr_channels *ch)
{
    return ch->t_us;
}

/*
 * Turn a sensor slot into one run by `drv` with state `self`. The slot's
 * name and signals are kept. Returns false (leaving the slot untouched) if
 * drv is incomplete or produces a different number of channels than the slot
 * has. Apps use lbr_bind() (lbr/node.h), which also records the reason.
 */
[[nodiscard]] bool lbr_sensor_use_driver(lbr_sensor_t *slot, const lbr_driver *drv, void *self);
