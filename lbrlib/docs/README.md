# LBRLib

LBRLib is the C library every Longhorn Baja Racing ESP32 runs on. It does for a race car's data
system what WPILib does for an FRC robot: you describe the car, bind your sensors, and write the
logic that matters. The library does the rest: sampling on a shared clock, the CAN bus, time
sync, the SD log, live telemetry, heartbeats, and making every failure visible.

```c
#include "lbr.h"

LBR_OUTPUT(ratio, "CVT/Ratio", F32, NONE);

static void cvt_periodic(lbr_cycle *c)
{
    lbr_out(c, &ratio, lbr_div(lbr_get(c, ENGINE_RPM), lbr_get(c, CVT_RPM)));
}

LBR_TASK(cvt, .rate_hz = 50, .periodic = cvt_periodic);
```

That task runs on the Central 50 times a second, and its output lands in the SD log next to the
raw data. If the engine speed sensor is unplugged, the powertrain board loses power, or the CAN
cable falls off, the ratio is logged as *unavailable, and why*: never as a made-up number.

## Guides

Start with the first one; the rest are references you come back to.

| Guide | What it covers |
| --- | --- |
| [Getting started](getting-started.md) | Build, test, and simulate with no hardware, then add a signal, a sensor, and a task, end to end (about 30 minutes) |
| [Concepts](concepts.md) | How the pieces fit: nodes as smart sensors, the Central as the brain, the shared clock, the catalog, and why nothing fails silently |
| [The catalog](catalog.md) | `car/catalog.toml`: nodes, signals, CAN frames, slots, and the generator |
| [Sensor nodes](nodes.md) | A node's `main.c`: `LBR_NODE_MAIN`, `lbr_bind`, `lbr_bind_io` |
| [Drivers](drivers.md) | Writing a driver for a new part, and wiring simple inputs with no code |
| [Car logic](logic.md) | `LBR_TASK`, `lbr_get`, `LBR_OUTPUT`: derived values and alarms on the Central |
| [Values and faults](values-and-faults.md) | `lbr_f32` and its statuses, the math helpers, and every way a fault is detected and reported |
| [Logging and data](logging.md) | The SD log, live streams, CSV export, and where replay is going |
| [Tools](tools.md) | `lbrgen.py`, the simulator, `lbr_decode.py`, `lbr_deploy.py` |
| [API reference](api.md) | Every public function and macro, on one page |

The wire formats are specified in [`docs/protocol.md`](../../docs/protocol.md) and
[`docs/can-ids.md`](../../docs/can-ids.md); putting firmware on the car is
[`docs/deploying.md`](../../docs/deploying.md).

## Where things live

```text
lbrlib/                    the library (generic: no car-specific names)
  components/lbr/          the app-facing API: lbr.h, values, car logic
  components/node/         the sensor-node runtime: LBR_NODE_MAIN, slots, acquisition
  components/sensors/      the driver kit and the built-in drivers (ADC/pulse/switch, NMEA GPS)
  components/...           protocol, CAN, time base, logging, transports, the Central's core
  tools/lbrgen.py          catalog.toml -> generated C, plans, and docs tables
  docs/                    these guides
car/                       this car, built on LBRLib
  catalog.toml             every node, signal, and CAN frame, declared once
  nodes/<node>/main/       each board's app (a few lines)
  logic/                   the Central's car logic tasks
dev/                       host tests, the simulator, the decoder (never on the car)
```

## Ground rules the library enforces

- **Nothing fails silently.** A missing value is a status with a reason, never a zero. Every
  dropped frame, overflowed queue, refused output, and late task is counted and reported.
- **One clock.** Every sample is stamped on the shared time base the Central distributes over
  CAN, never on when it happened to arrive.
- **The SD log is the truth.** A slow or dead radio link never costs a byte of the log.
- **Declare once.** A signal is one line in `car/catalog.toml`; everything else is generated
  and checked.
- **Pure logic, thin hardware glue.** Everything except `*_esp.c` runs on a laptop, so the host
  tests and the simulator run exactly the code that is flashed.
