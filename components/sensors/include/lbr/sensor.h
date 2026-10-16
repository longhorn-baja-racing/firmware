#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lbr/protocol.h"

/*
 * Generic sensor interface.
 *
 * Concrete drivers (IMU, RPM pickup, speed, temperatures, fuel level, ...)
 * implement this vtable so a node's acquisition loop stays independent of the
 * specific parts chosen. PARTS ARE NOT SELECTED YET — keep every driver behind
 * this interface so swapping chips does not touch acquisition/logging/protocol.
 */

typedef struct lbr_sensor lbr_sensor_t;

struct lbr_sensor {
    const char *name;

    int (*init)(lbr_sensor_t *self);

    /* Produce up to `cap` samples for this acquisition tick. Returns the count,
     * or <0 on error. Channels with no fresh/valid reading MUST be reported as
     * samples with `valid = false` rather than omitted or zeroed. */
    int (*sample)(lbr_sensor_t *self, lbr_sample_t *out, size_t cap);

    /* Packed sensor status/self-test bits (meaning is per-driver, documented). */
    uint32_t (*status)(lbr_sensor_t *self);

    void *ctx;
};
