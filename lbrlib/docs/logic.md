# Car logic

Car logic turns the bus into answers: a CVT ratio, the hottest brake, an overheat alarm, a lap
time. It runs on the Central as **tasks**, small periodic functions that read any signal on the
car and write named **outputs**. Outputs go into the SD log next to the raw data and onto the
USB/WiFi live stream, so the dashboard plots them like any signal.

```c
#include "car_logic.h"

LBR_OUTPUT(hottest_brake, "Brakes/HottestC", F32, DEG_C);
LBR_OUTPUT(brake_alarm, "Alarms/BrakeOverheat", BOOL, NONE);

static bool s_hot;

static void brakes_periodic(lbr_cycle *c)
{
    lbr_f32 front = lbr_max(lbr_get(c, TEMP_BRAKE_FL), lbr_get(c, TEMP_BRAKE_FR));
    lbr_f32 back  = lbr_max(lbr_get(c, TEMP_BRAKE_BL), lbr_get(c, TEMP_BRAKE_BR));
    lbr_f32 hot   = lbr_max(front, back);

    lbr_out(c, &hottest_brake, hot);
    lbr_out_bool(c, &brake_alarm, lbr_above(&s_hot, hot, 300.0f, 270.0f));
}

LBR_TASK(brakes, .rate_hz = 10, .periodic = brakes_periodic);
```

## Adding a task

1. Write it in its own file in `car/logic/` (`#include "car_logic.h"`).
2. Declare it in `car/logic/include/car_logic.h`: `extern const lbr_task brakes;`
3. Add it to the list in `car/logic/tasks.c`; tasks run in that order each tick:
   `const lbr_task_list car_logic = LBR_TASK_LIST(&vehicle, &cvt, &brakes, &oil);`
4. Add the file to `SRCS` in `car/logic/CMakeLists.txt` (the host build picks it up by itself).

`ctest` checks the task list (`test_plans`), and the simulator runs it: add a scenario with
`expect out:<key> ...` checks (see `dev/sim/scenarios/car_logic.sim`).

## Tasks

```c
LBR_TASK(name, .rate_hz = 50, .periodic = fn, .init = optional_fn);
```

| | |
| --- | --- |
| `.rate_hz` | 1, 2, 4, 5, 10, 20, 25, 50, or 100 (divides `LBR_LOGIC_TICK_HZ` = 100) |
| `.periodic(c)` | called every 1/rate seconds, on a grid of the shared clock |
| `.init(c)` | optional, once before the first `periodic` |

A task must be quick and self-contained: no blocking, no I/O, no hardware. Everything it needs
comes in through `lbr_get` and everything it produces goes out through `lbr_out`. Keep its state
in `static` variables in its file. These rules are also what makes a task replayable from a log
later ([logging](logging.md#replay)).

A task that runs late (its next run was already due) is counted, and it rejoins the grid
instead of running twice. A bad task list (a rate that does not fit, two tasks with one name, no
`periodic`) stops the car logic, never the data path: the Central reports `CONFIG_INVALID`
(argument 1) with the reason in its log.

## Reading signals

```c
lbr_f32  x = lbr_get(c, ENGINE_RPM);      /* any catalog signal, by its catalog name */
lbr_bool b = lbr_get_bool(c, COCKPIT_LINK);
lbr_i32  i = lbr_get_i32(c, GPS_LATITUDE);
```

You get the latest value with its **status** and **age**:

| Status | When |
| --- | --- |
| `LBR_OK` | a fresh, valid value |
| `NOT_FITTED`, `SENSOR_FAULT`, `BUS_ERROR`, ... | the sensor node says why it has no value |
| `NO_DATA` | nothing received yet |
| `STALE` | nothing new for 3 of its frame periods (30 ms for a 100 Hz signal) |
| `NODE_LOST` | its node dropped off the bus |
| `NODE_MISSING` | its node was never heard this session |
| `BAD_MATH` | a NaN came in |

Integer, status, and bool signals read as their numeric value through `lbr_get`.

## Doing math

Work on the `lbr_f32` values directly and the status follows the data:

```c
lbr_f32 ratio = lbr_div(lbr_get(c, ENGINE_RPM), lbr_get(c, CVT_RPM));
```

If either speed is unavailable, `ratio` is unavailable with that reason; if the CVT is not
turning, it is `BAD_MATH` (division by zero). It never becomes a number that looks real. The
helpers (`lbr_add`, `lbr_sub`, `lbr_mul`, `lbr_div`, `lbr_scale`, `lbr_min`, `lbr_max`,
`lbr_abs`, `lbr_clamp`, `lbr_gt`, `lbr_lt`, `lbr_and`, `lbr_or`, `lbr_not`, `lbr_above`,
`lbr_below`, `lbr_lowpass`) are in [values and faults](values-and-faults.md).

To use a plain number, check first, or say what "unknown" means:

```c
if (lbr_ok(ratio)) { ... ratio.v ... }
float shown = lbr_or_else(ratio, 0.0f);
```

## Outputs

```c
LBR_OUTPUT(var, "Group/Name", TYPE, UNIT);   /* TYPE: F32 or BOOL; UNIT: a catalog unit */

lbr_out(c, &var, value);                      /* an lbr_f32 */
lbr_out_bool(c, &var, flag);                  /* an lbr_bool */
```

Every write is logged with its status: an unavailable value is logged as unavailable, with the
reason, so the log says *why* a derived value is missing. Keys are `Group/Name`, ASCII, with no
spaces, commas, or quotes, at most 48 characters; group them (`Alarms/...`, `Brakes/...`) so the
dashboard can tree them. Up to 64 outputs; one that cannot be registered (a bad key, too many)
is counted in `outputs_refused`, never written silently wrong.

An output is logged on every write. At 50 Hz that is 750 bytes a second, small next to the raw
bus, but write slow-changing values from slower tasks.

## Where the tasks run

The Central's gateway core feeds every decoded sample to the logic runner (`lbr_logic_offer`),
and a 100 Hz tick (`lbr_gw_core_logic_tick`) runs the tasks that are due. Tasks see the bus as of
that tick. The runner is pure C, so the simulator and the host tests run it exactly as the
Central does.
