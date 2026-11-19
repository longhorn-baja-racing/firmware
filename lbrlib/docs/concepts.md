# Concepts

## The car as a robot

If you know FRC, the car maps onto it closely:

| FRC (WPILib / AdvantageKit) | LBRLib |
| --- | --- |
| The roboRIO runs the robot code | The **Central** runs the car logic (`car/logic/`) |
| CAN devices (motor controllers, encoders) report their values | **Sensor nodes** sample their sensors and publish them on CAN |
| `Robot.periodic()` and subsystems | `LBR_TASK`s at their own rates |
| `Logger.recordOutput` | `LBR_OUTPUT` + `lbr_out` |
| WPILOG files, AdvantageScope | The SD log (`.LBR`), the dashboard app |

**Sensor nodes** (`imu`, `powertrain`, `driver_input`, `temperature`, `stress`, and the four
`corner` hubs) are smart sensors. Each owns a few sensors, samples them on the shared clock,
and publishes typed values onto the CAN bus. They hold no car logic, and you write almost no
code for them: a node's `main.c` only says which part is on which slot.

**The Central** is the brain and the recorder. It hears every frame on the bus, writes the
authoritative SD log, keeps the shared clock, carries the GPS, runs the car logic, and feeds
every live link: LoRa in the race, USB and WiFi in the pit, BLE for walk-up checks, and the
cockpit display over WiFi.

**The cockpit** is the driver's display. It is off the CAN bus and gets what it shows from the
Central over WiFi.

## The catalog: describe the car once

`car/catalog.toml` is the single description of the car: every node and its id, every signal
(id, type, unit), and how each one travels on CAN (which frame, which bytes, what scale, how
fast). `lbrlib/tools/lbrgen.py` generates the C tables, every node's plan, and the docs tables
from it, and checks it while it does: ids are unique, fields fit their frames, rates divide a
second, every slot is filled, and the whole bus fits its budget (in `test_plans`).

Everything downstream is schema-driven: the decoder, the CSV export, and the dashboard read
the catalog rather than hard-coding layouts. See [the catalog](catalog.md).

## Slots

A node's sensors are grouped into **slots**: the channels one part produces, read together at
one rate. The powertrain has `speeds` (engine and CVT speed at 100 Hz) and `fuel` (1 Hz). A
slot's signals travel in frames that node sends at the slot's rate, so every sample is
published exactly once. In `main.c` you bind a driver to each slot that has a part; a slot with
no part reports `NOT_FITTED`, so the data shows as "expected, not fitted" rather than missing.

## One clock

Every sample is timestamped on a **shared time base**. The Central is the time master and
broadcasts a two-step time sync on CAN ten times a second; every node disciplines its clock to
it (slewing small errors, stepping large ones). Nodes sample on a grid of that clock: sample k
of a 100 Hz slot is taken at exactly k x 10 ms, and its frame carries k. So a receiver knows the
true sample time, and CAN latency never becomes measurement time.

## Nothing fails silently

The rule that shapes the whole library: a missing value is never a number, and nothing is
dropped without being counted.

- A channel with no part, a dead sensor, or a bad read is published as **unavailable with a
  reason** (`NOT_FITTED`, `SENSOR_FAULT`, `BUS_ERROR`, ...). On CAN the reason travels in
  reserved raw values, so it can never be mistaken for zero.
- The Central knows which nodes the catalog expects. A node that never shows up is reported
  `NODE_MISSING`; one that goes quiet is `NODE_LOST`, and `NODE_FOUND` when it is back.
- In car logic every value is an `lbr_f32` with a status and an age, and the math helpers pass
  the status through. A derived value of a lost node's data is logged as `NODE_LOST`, not as a
  stale number.
- Every queue overflow, dropped frame, refused output, late task, and bus error is counted and
  reported in heartbeats and the log.

See [values and faults](values-and-faults.md) for the full list.

## The log is the truth

The Central's SD log holds every frame on the bus, byte for byte, with its receive time, plus
events, health, the session configuration, and the car logic outputs. It is written through a
bounded queue; a slow or absent radio link never costs it anything. Records carry a CRC, and a
log cut by a power loss is recovered up to the last complete record. The CSV export, the live
links, and the cockpit are all views of the same records. See [logging](logging.md).

## Layers, not a web

LBRLib is a stack of components, and each may use only the ones below it:

```text
4  roles        lbr_node  lbr_central  lbr_cockpit      LBR_*_MAIN: wiring a board together
3  ESP32 glue   lbr_hw (CAN, clock, inputs, GPS, OTA, SD)   lbr_links (USB, LoRa, WiFi, BLE)
2  core         lbr_core: sensors, node/Central/cockpit logic, car logic   (pure C)
1  base         lbr_base: protocol, CAN codec, health, time servo, log format   (pure C)
0  catalog      the car's generated catalog
```

`dev/ci/check_layers.py` (in `ctest` and CI) fails on any include that points up, or from one
role into another, and on any ESP-IDF header in the pure layers. So a new feature cannot quietly
tie the library into a knot.

The pure layers take their hardware as tables of functions, so the host tests and the simulator
run exactly the code that is flashed, on a laptop, under sanitizers. A role is only the wiring:
FreeRTOS tasks and ESP32 drivers around the pure core, and an app picks its role in one line.

## Ordering and replay

Replay (re-running the car logic on a recorded log, the way AdvantageKit does) needs every input
to the logic to be in the log, in the order the logic saw it. LBRLib gets there in two parts:

- **Between boards, the CAN bus is the order.** Arbitration puts every frame in one sequence,
  and the Central records all of them. Sensor nodes are treated as I/O devices, so their frames
  are the inputs; their firmware does not need replaying.
- **Inside the Central, one logic thread.** The car logic tasks run one at a time on one tick,
  reading only through `lbr_get()` and writing only through `lbr_out()`. Background work (CAN
  receive, radios, the SD writer) only feeds queues.

Car logic already follows these rules, so tasks you write today will replay unchanged. The
remaining steps, which run the whole Central core on one cycle, log its non-CAN inputs, and add
the `lbr_replay` tool and the `.lbrlog` format, are on the roadmap ([logging](logging.md#replay)).
