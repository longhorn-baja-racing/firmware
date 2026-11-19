#pragma once

/*
 * LBRLib: one include for app code.
 *
 *     #include "lbr.h"
 *
 *   Sensor node apps   LBR_NODE_MAIN, lbr_bind, lbr_bind_io, lbr_slot  (lbr/node_app.h)
 *   Part drivers       lbr_driver, lbr_ch_f32, lbr_ch_missing, ...    (lbr/driver.h)
 *   Car logic          LBR_TASK, LBR_OUTPUT, lbr_get, lbr_out          (lbr/logic.h)
 *   Values             lbr_f32, lbr_ok, lbr_or_else, lbr_div, lbr_above   (lbr/value.h)
 *   The catalog        LBR_SIG_<NAME>, LBR_NODE_<NAME>                 (generated)
 *
 * Guides: lbrlib/docs/README.md.
 */

#include "lbr/driver.h"
#include "lbr/logic.h"
#include "lbr/node_app.h"
#include "lbr/nodes.h"
#include "lbr/protocol.h"
#include "lbr/value.h"
