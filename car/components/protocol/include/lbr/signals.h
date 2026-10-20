#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Stable signal identifiers (schema-driven protocol), generated from
 * signals.def. These values are part of the wire/log/CAN protocol and MUST
 * stay synchronized with lbr-dashboard.
 */
typedef enum {
    LBR_SIG_UNKNOWN = 0x0000,
#define LBR_SIGNAL(name, id, type, unit, desc) LBR_SIG_##name = (id),
#include "lbr/signals.def"
#undef LBR_SIGNAL
} lbr_signal_id_t;
