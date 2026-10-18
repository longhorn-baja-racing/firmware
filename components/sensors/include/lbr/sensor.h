#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/protocol.h"

/*
 * Generic sensor driver interface.
 *
 * A driver owns one physical part (or one bus device) and produces a fixed,
 * ordered list of channels, each a catalog signal. The node runtime calls
 * read() at the configured rate and turns each reading into a timestamped,
 * sequenced sample; drivers never deal with time sync, sequence numbers,
 * CAN, or logging. Swapping a part means swapping the driver, nothing else.
 *
 * Rules every driver follows:
 *   - report sensor-frame values in the catalog unit (no mounting transforms);
 *   - a channel without a fresh, trustworthy value is reported with
 *     valid = false and a reason, never as zero;
 *   - read() must not block longer than one sample period.
 */

typedef struct {
    bool                 valid;
    lbr_unavail_reason_t reason; /* when !valid */
    union {
        float    f32;
        int32_t  i32;
        uint32_t u32;
        bool     b;
    } v;
} lbr_reading_t;

typedef struct lbr_sensor lbr_sensor_t;

struct lbr_sensor {
    const char            *name;    /* driver/part name, recorded in session config */
    const lbr_signal_id_t *signals; /* channel order of read() output */
    size_t                 n_signals;

    /* Bring up the part. Return 0 on success; on failure the runtime reports
     * every channel as SENSOR_FAULT and retries init periodically. */
    int (*init)(lbr_sensor_t *self);

    /* Fill out[0..n_signals-1] for this tick. `t_us` is the acquisition time
     * (shared time base). Return 0, or <0 if the part stopped responding (the
     * runtime then re-inits it). */
    int (*read)(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out);

    /* Optional: true if the part's calibration is valid. NULL = no
     * calibration concept for this part (reported as calibrated). */
    bool (*is_calibrated)(lbr_sensor_t *self);

    /* True for drivers that synthesize data. The node then raises
     * LBR_HF_SIM_SENSORS so no log mistakes it for a measurement. */
    bool simulated;

    void *ctx;
};

#define LBR_SENSOR_MAX_SIGNALS 16u

/*
 * Placeholder for a channel group whose part is not chosen or not fitted:
 * init succeeds and every channel reads valid = false, NOT_FITTED. Lets a
 * node publish its full channel list (so the dashboard sees the channels are
 * expected but absent) before any hardware exists.
 */
void lbr_sensor_unconfigured(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                             size_t n_signals);

/*
 * Bench simulator: plausible synthetic waveforms per channel (sine for
 * floats, slow ramps for counters, square wave for booleans). Marked
 * simulated. Never ship a race configuration with this driver.
 */
void lbr_sensor_sim(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                    size_t n_signals);
