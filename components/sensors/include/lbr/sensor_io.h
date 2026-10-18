#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/sensor.h"

/*
 * Part-independent ESP32 inputs: on-chip ADC, pulse frequency, and digital
 * inputs. Most Baja sensors reach the node as one of these whatever the part
 * turns out to be (potentiometers, pressure transducers, fuel senders,
 * thermistors, hall/VR pickups behind a conditioner, switches, buttons), so
 * a slot can be wired by configuration alone, with no driver code.
 *
 * A slot is described by one ASCII spec string (a node Kconfig option), one
 * channel per slot signal, in the slot's order, separated by ';':
 *
 *   chan := '-'                       not fitted (NOT_FITTED)
 *         | 'adc'   { key=value }     F32 signal
 *         | 'pulse' { key=value }     F32 signal
 *         | 'din'   { key=value }     BOOL signal, or U32 bitmask
 *
 *   adc    gpio=N (ADC1 pin, 32..39)  avg=1..64 (4)
 *          curve=lin (default): mv0= v0= mv1= v1=   two-point calibration
 *          curve=ntc: r25= beta= rpull= vsup=        NTC to ground, pull-up
 *                     to vsup (mV); result in degC
 *          lo= hi=     electrical window in mV; outside = SENSOR_FAULT
 *                      (open or shorted sensor)
 *          min= max=   physical range; outside = OUT_OF_RANGE
 *   pulse  gpio=N  scale=  (value per Hz: 60/ppr for rpm, circumference /
 *                  teeth for m/s)  stall_ms=1..60000 (2000)
 *          edge=rise|fall (rise)  pull=up|down|none (none)
 *   din    gpio=N or gpio=A,B,C (bit i = i-th pin; U32 signals only)
 *          active=low|high (low)  pull=up|down|none (up)  debounce_ms=0..1000 (10)
 *
 * Example (driver_inputs pedals: throttle pot, two 0.5-4.5 V transducers
 * behind 2:3 dividers):
 *   "adc gpio=34 mv0=310 v0=0 mv1=2980 v1=100 lo=150 hi=3150;
 *    adc gpio=35 mv0=333 v0=0 mv1=3000 v1=13790 lo=150 hi=3150;-"
 *
 * The spec is calibration: it is recorded verbatim in the session config.
 * Parsing, conversion, pulse rate, and debounce are pure and host-tested;
 * sensor_io_esp.c is the hardware side.
 */

#define LBR_IO_MAX_DIN_PINS 8u
#define LBR_IO_SPEC_MAX     512u

typedef enum {
    LBR_IO_NONE = 0,
    LBR_IO_ADC,
    LBR_IO_PULSE,
    LBR_IO_DIN,
} lbr_io_kind_t;

typedef enum {
    LBR_IO_PULL_NONE = 0,
    LBR_IO_PULL_UP,
    LBR_IO_PULL_DOWN,
} lbr_io_pull_t;

typedef struct {
    lbr_io_kind_t kind;
    union {
        struct {
            int8_t  gpio;
            uint8_t avg;
            bool    ntc;
            float   mv0, v0, mv1, v1;          /* curve=lin */
            float   r25, beta, rpull, vsup_mv; /* curve=ntc */
            float   lo_mv, hi_mv;              /* electrical window */
            float   min, max;                  /* physical range */
        } adc;
        struct {
            int8_t        gpio;
            float         scale;
            uint32_t      stall_ms;
            bool          falling;
            lbr_io_pull_t pull;
        } pulse;
        struct {
            int8_t        gpio[LBR_IO_MAX_DIN_PINS];
            uint8_t       n_gpio;
            bool          active_high;
            lbr_io_pull_t pull;
            uint32_t      debounce_ms;
        } din;
    } u;
} lbr_io_chan_t;

/*
 * Parse `spec` for a slot of `n` signals into chans[0..n-1], checking every
 * channel kind against the catalog type of its signal. Returns 0, or <0 with
 * a one-line reason in `err`. An empty spec (or NULL) is an error: callers
 * use the unconfigured driver for slots without one.
 */
int lbr_io_parse(const char *spec, const lbr_signal_id_t *signals, size_t n, lbr_io_chan_t *chans,
                 char *err, size_t err_len);

/* ADC: convert a calibrated pin voltage (mV) into a reading. */
void lbr_io_adc_convert(const lbr_io_chan_t *c, float mv, lbr_reading_t *out);

/*
 * Pulse rate from an edge counter. The capture side keeps a free-running
 * edge count and the local time of the latest edge; read() hands both over
 * with the current local time. Frequency is measured edge to edge (exact at
 * any speed), decays as 1/elapsed while no edge arrives, and reads 0 after
 * stall_ms without edges.
 */
typedef struct {
    uint32_t ref_count;
    uint64_t ref_edge_us;
    bool     have_ref;
    float    hz;
} lbr_io_pulse_t;

float lbr_io_pulse_update(lbr_io_pulse_t *st, uint32_t count, uint64_t edge_us, uint64_t now_us,
                          uint32_t stall_ms);

/* Digital debounce: a new raw state is accepted once it has been stable for
 * debounce_ms. Seeded from the first sample. */
typedef struct {
    uint32_t stable;
    uint32_t pending;
    uint64_t since_us;
    bool     seeded;
} lbr_io_din_t;

uint32_t lbr_io_din_update(lbr_io_din_t *st, uint32_t raw, uint64_t now_us, uint32_t debounce_ms);

/*
 * Hardware driver (sensor_io_esp.c). Fills `out` for the slot; the spec
 * string must outlive the sensor. A spec that does not parse still yields a
 * sensor, whose init() fails, so the runtime reports SENSOR_INIT_FAIL and
 * every channel as SENSOR_FAULT instead of silently looking not fitted.
 */
void lbr_sensor_io(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                   size_t n_signals, const char *spec);
