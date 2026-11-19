# API reference

Everything app code uses comes in with one include:

```c
#include "lbr.h"
```

Headers are in `lbrlib/components/*/include/lbr/`; each has the full contract in its comments.

## Sensor node apps (`lbr/node_app.h`, `lbr/node.h`)

| | |
| --- | --- |
| `LBR_NODE_MAIN(&lbr_plan_<node>, .init = fn)` | the node's whole `app_main`: build, bind (`init`), validate, run. Also `.on_frame`, `.on_frame_ctx` for `listen_all` plans |
| `void init(lbr_node *n)` | your setup function: bind parts to slots |
| `lbr_bind(n, SLOT, &driver, &state)` | run a slot with a part driver; `state` must outlive the node |
| `lbr_bind_io(n, SLOT, spec)` | run a slot with the configurable ADC/pulse/switch inputs; empty spec = unbound |
| `lbr_slot(n, SLOT)` | a slot's raw `lbr_sensor_t` (NULL, and the config invalid, if bound twice or out of range) |
| `lbr_node_emit_event(code, arg, fault)` | publish an event or fault from app code |
| `<NODE>_SLOT_<NAME>`, `lbr_plan_<node>` | generated from the catalog (`<node>_plan.h`) |

## Drivers (`lbr/driver.h`)

| | |
| --- | --- |
| `lbr_driver {part, channels, start, read, calibrated}` | a part driver |
| `int start(void *self)` | bring the part up: 0, or <0 |
| `int read(void *self, lbr_channels *ch)` | one sample of every channel: 0, or <0 if the part stopped answering |
| `lbr_ch_f32(ch, i, v)`, `lbr_ch_i32`, `lbr_ch_u32`, `lbr_ch_bool` | set channel i |
| `lbr_ch_missing(ch, i, why)`, `lbr_ch_missing_all(ch, why)` | no value, and why (`lbr_unavail_reason_t`) |
| `lbr_ch_count(ch)`, `lbr_ch_time_us(ch)` | channel count; sample time on the shared clock |

`lbr_unavail_reason_t`: `LBR_UNAVAIL_NOT_FITTED`, `_NO_DATA`, `_SENSOR_FAULT`, `_OUT_OF_RANGE`,
`_NOT_CALIBRATED`, `_BUS_ERROR`, `_UNSPECIFIED`.

## Car logic (`lbr/logic.h`)

| | |
| --- | --- |
| `LBR_TASK(name, .rate_hz = N, .periodic = fn, .init = fn)` | a task; `N` divides 100 |
| `LBR_OUTPUT(var, "Group/Name", F32 or BOOL, UNIT)` | an output |
| `lbr_get(c, NAME)`, `lbr_get_bool(c, NAME)`, `lbr_get_i32(c, NAME)` | the latest value of catalog signal `NAME`, with status and age |
| `lbr_signal(c, LBR_SIG_<NAME>)` (and `_bool`, `_i32`) | the same, by id |
| `lbr_out(c, &var, lbr_f32)`, `lbr_out_bool(c, &var, lbr_bool)` | write an output (logged with its status) |
| `lbr_now_us(c)`, `lbr_dt_s(c)` | this run's shared time; the task's period in seconds |
| `LBR_LOGIC_TICK_HZ` | 100: the fastest a task runs |

## Values (`lbr/value.h`)

| | |
| --- | --- |
| `lbr_f32`, `lbr_i32`, `lbr_bool` | `{v, st, age_ms}` |
| `lbr_status` | `LBR_OK`, `LBR_NOT_FITTED`, `LBR_NO_DATA`, `LBR_SENSOR_FAULT`, `LBR_OUT_OF_RANGE`, `LBR_NOT_CALIBRATED`, `LBR_BUS_ERROR`, `LBR_UNSPECIFIED`, `LBR_STALE`, `LBR_NODE_LOST`, `LBR_NODE_MISSING`, `LBR_BAD_MATH` |
| `lbr_f32_of(v)`, `lbr_i32_of(v)`, `lbr_bool_of(v)`, `lbr_f32_missing(st)` | make one |
| `lbr_ok(x)`, `lbr_or_else(x, fallback)` | check one; use one with an explicit default |
| `lbr_add`, `lbr_sub`, `lbr_mul`, `lbr_div`, `lbr_scale`, `lbr_offset`, `lbr_abs`, `lbr_min`, `lbr_max`, `lbr_clamp` | math; the status flows through |
| `lbr_gt`, `lbr_lt` | compare to a limit, giving an `lbr_bool` |
| `lbr_and`, `lbr_or`, `lbr_not` | logic that claims only what it knows |
| `lbr_above(&state, x, on, off)`, `lbr_below(&state, x, on, off)` | alarms with hysteresis |
| `lbr_lowpass(&filter, x, alpha)` with `static lbr_filter filter;` | first-order low-pass |
| `lbr_i32_to_f32(x)` | |
| `lbr_status_name(st)`, `lbr_status_from_reason(reason)` | |

## The catalog (generated)

| | |
| --- | --- |
| `LBR_SIG_<NAME>` | signal ids |
| `LBR_SIGIDX_<NAME>`, `LBR_SIGNAL_COUNT` | catalog indexes, as constants |
| `LBR_NODE_<NAME>`, `LBR_NODE_CENTRAL`, `LBR_NODE_COCKPIT` | node ids and the two library roles |
| `lbr_signal_info(id)`, `lbr_signal_index(id)`, `lbr_signal_count()`, `lbr_signal_at(i)` | signal metadata (name, type, unit, description) |
| `lbr_node_name(id)`, `lbr_node_about(id)`, `lbr_node_count()`, `lbr_node_id_at(i)`, `lbr_node_is_can(id)` | node metadata |
| `lbr_frame_for_signal(id, &field)`, `lbr_frame_def(can_id)` | where a signal travels |
| `lbr_catalog_hash()` | the catalog's identity, in every heartbeat and log |

## Limits

| | |
| --- | --- |
| Slots per node | 16 (`LBR_NODE_MAX_SENSORS`) |
| Channels per slot | 16 |
| Fields per CAN frame | 8 |
| Acquisition rate | 1..1000 Hz, dividing 1000000 |
| Car logic tasks | 16 (`LBR_LOGIC_MAX_TASKS`), 1..100 Hz |
| Car logic outputs | 64 (`LBR_LOGIC_MAX_OUTPUTS`), keys up to 48 characters |
