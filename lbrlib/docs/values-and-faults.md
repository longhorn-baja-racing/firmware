# Values and faults

## Values with a status

Car logic never handles a bare `float`. Every value is one of:

```c
typedef struct { float   v; lbr_status st; uint32_t age_ms; } lbr_f32;
typedef struct { int32_t v; lbr_status st; uint32_t age_ms; } lbr_i32;
typedef struct { bool    v; lbr_status st; uint32_t age_ms; } lbr_bool;
```

`v` is only meaningful when `st == LBR_OK`; otherwise it is 0, and `st` says why the value is
missing. `age_ms` is how long ago the newest measurement behind it arrived.

| `lbr_status` | Meaning |
| --- | --- |
| `LBR_OK` | valid |
| `LBR_UNSPECIFIED` | unavailable for an unstated reason |
| `LBR_NOT_FITTED` | no part on that channel |
| `LBR_NO_DATA` | the part is alive but has nothing fresh, or nothing was received yet |
| `LBR_SENSOR_FAULT` | the part failed, will not start, or reads open/shorted |
| `LBR_OUT_OF_RANGE` | outside its valid range, or too big for its CAN field |
| `LBR_NOT_CALIBRATED` | raw reading, no calibration |
| `LBR_BUS_ERROR` | an I2C/SPI/ADC transaction failed |
| `LBR_STALE` | stopped updating (3 frame periods) |
| `LBR_NODE_LOST` | its node dropped off the bus |
| `LBR_NODE_MISSING` | its node was never heard this session |
| `LBR_BAD_MATH` | division by zero, or a NaN or infinite result |

`lbr_status_name(st)` gives the name.

### Making and checking

| | |
| --- | --- |
| `lbr_f32_of(v)`, `lbr_i32_of(v)`, `lbr_bool_of(v)` | a valid value |
| `lbr_f32_missing(st)` | an unavailable one |
| `lbr_ok(x)` | true when x is usable (any of the three types) |
| `lbr_or_else(x, fallback)` | x's value, or `fallback`: the default is explicit on purpose |

### Math

The status flows through like a NaN, but with a reason: the first operand's problem wins, then
the second's. Results that would be NaN or infinite become `LBR_BAD_MATH`. The age is the
oldest input's.

| Function | |
| --- | --- |
| `lbr_add(a, b)`, `lbr_sub`, `lbr_mul`, `lbr_div` | `lbr_div` by zero is `BAD_MATH` |
| `lbr_scale(a, k)`, `lbr_offset(a, k)` | a x k, a + k |
| `lbr_abs(a)`, `lbr_min(a, b)`, `lbr_max(a, b)`, `lbr_clamp(a, lo, hi)` | |
| `lbr_gt(a, limit)`, `lbr_lt(a, limit)` | an `lbr_bool`, unavailable (and never true) when a is |
| `lbr_i32_to_f32(a)` | |

### Logic and alarms

| Function | |
| --- | --- |
| `lbr_and(a, b)` | false (and OK) as soon as either side is known false; else unavailable if either is |
| `lbr_or(a, b)` | true (and OK) as soon as either side is known true; else unavailable if either is |
| `lbr_not(a)` | |
| `lbr_above(&state, x, on, off)` | alarm: on above `on`, off below `off` (hysteresis, `off < on`) |
| `lbr_below(&state, x, on, off)` | alarm: on below `on`, off above `off` (`off > on`) |
| `lbr_lowpass(&filter, x, alpha)` | first-order filter; `static lbr_filter filter;`. An unavailable input passes through and restarts it |

An alarm whose input is unavailable is *unknown* (it carries the input's status), never "off",
and it keeps its state for when the input returns. `lbr_and` lets a known fact settle the
answer: "engine running AND oil low" is a plain false when the engine is known to be off, even
if the oil sensor is gone.

## Every fault, and where it shows

| Fault | Detected by | Shows as |
| --- | --- | --- |
| A slot with no part | the node (unbound driver) | channels `NOT_FITTED` on CAN and in the log; the slot's `sensor_status` bit is clear, so the node reports `DEGRADED` with the sensor error flag |
| One channel without a value (open wire, out of range, a bad read) | the driver or the input spec | that channel's reason on CAN; the slot's `sensor_status` bit clears and the node reports `DEGRADED` until a complete reading |
| A part that will not start or stops answering | the node runtime | `SENSOR_INIT_FAIL` fault (argument: slot), channels `SENSOR_FAULT`, retried every 2 s, `SENSOR_RECOVERED` when back |
| A bad reading | the driver | the channel's reason (`BUS_ERROR`, `OUT_OF_RANGE`, ...) on CAN |
| A value too big for its CAN field | the node | `OUT_OF_RANGE` on CAN, counted in `rejected_count` |
| A node that never powers up | the Central (the catalog lists it) | `NODE_MISSING` fault (argument: node id); its values read `NODE_MISSING` |
| A node that goes quiet | the Central (3 s without a frame) | `NODE_LOST` fault; its values read `NODE_LOST`; `NODE_FOUND` when back |
| A value that stopped updating | the car logic runner | `STALE` |
| A configuration mistake (bad slot binding, wrong channel count, bad plan) | the node at boot | `CONFIG_INVALID` fault and heartbeat state; CAN and the heartbeat keep running |
| A bad car logic task list | the Central at boot | `CONFIG_INVALID` fault (argument 1); the data path keeps running |
| Unset CAN pins | the node | `CAN_PINS_UNSET` |
| CAN bus-off | the CAN driver | `CAN_BUS_OFF` (recovers by itself), counts in the heartbeat |
| A full queue, a dropped frame | everywhere | counted: queue high-water marks, overflow and drop counters in every heartbeat; `LOSS` records in the log with exact counts |
| Central: a car logic output refused (bad or duplicate key, too many), a LoRa event evicted, a cockpit datagram lost | the Central | counted in its heartbeat: refused outputs in `overflow_count`, live-link drops in `transport_error_count` |
| A late car logic task | the runner | counted (`late`), rejoins the grid |
| SD card missing, write error | the Central | `SD_MISSING`, `SD_WRITE_ERROR`; retried every 5 s, `SD_MOUNTED` when back |
| Watchdog, brownout, panic | the next boot | the reset reason in the `BOOT` event and every heartbeat |
| Clock trouble | every node | `TIME_STEP` events, sync error in the heartbeat; samples marked unsynced until the clock is on the shared base |

Heartbeats go out every second from every node (state `RUN`, `DEGRADED`, `CONFIG_INVALID`, or
`FAULT`, with error flags), and the full health record (versions, uptime, rates, counters, bus
errors, per-slot sensor and calibration state, clock error) follows in 12 CAN pages. The Central
logs them all, and LoRa carries the critical subset in the race.
