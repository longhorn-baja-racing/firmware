# LBR Vehicle Firmware

Embedded telemetry and data-logging firmware for Longhorn Baja Racing vehicles.

The 2026 architecture ([`DAQ-WIRING.md`](DAQ-WIRING.md)) is a **distributed CAN network of
ESP32 nodes**. Six wired controllers each own a different set of sensors and publish their
measurements onto a shared **CAN bus**; four battery-powered corner nodes send suspension IMU
data over 5 GHz WiFi to the IMU Controller, which forwards it onto CAN. The **Central
Controller** records the authoritative log, keeps the shared clock, carries the GPS, and drives
the live telemetry links, including the in-cockpit dashboard over WiFi. The firmware is the vehicle-side half of the telemetry system; the
companion desktop application lives in
[`longhorn-baja-racing/lbr-dashboard`](https://github.com/longhorn-baja-racing/lbr-dashboard).

The recommended runtime is **C/C++ on ESP-IDF** (Espressif's official framework), using the
ESP32's built-in **TWAI** controller for CAN with an external CAN transceiver.

The 2026 vehicle carries five wired sensor controllers (**IMU, powertrain, driver input,
temperature, stress**), the **Central** controller, four **corner** nodes, and the cockpit
**dashboard**.

## Project status

The framework is in place and tested; sensor parts and pins are not chosen yet. To build,
test, simulate, or flash, see [Getting started](#getting-started).

- **Shared libraries** (`car/components/`): protocol, signal catalog and CAN frame table, CAN
  codec and TWAI adapter, CAN-distributed time sync, recoverable SD logging, heartbeat/health,
  sensor framework, grid-locked acquisition, node runtime, the corner and dashboard WiFi links,
  live transports (USB, LoRa, WiFi, BLE), and the Central's logic. The hardware-independent
  parts run in host tests under ASan/UBSan.
- **Node apps** (`car/nodes/`): one per node of DAQ-WIRING.md. Every node publishes its full
  channel list today; channels without a chosen part report `valid = false / NOT_FITTED` until a
  driver is bound (or simulated data with `CONFIG_LBR_SENSOR_SIM`, bench only). Analog, pulse,
  and switch sensors and any NMEA GPS need no driver code: they are wired and calibrated from
  menuconfig ([`docs/sensors.md`](docs/sensors.md)).
- **CI** runs host tests (gcc + clang), formatting, static analysis, the golden-fixture
  replay, and builds every node twice (shipped config and a link-coverage overlay).

Open, by design: part-specific drivers (IMU, strain amplifier, thermocouples; #10), per-node GPIO and CAN bus
parameters (#15), and agreeing the canonical spec location with `lbr-dashboard` (#14). All
pins default to `-1` in Kconfig and the firmware refuses to drive an unset pin.

## 2026 goals

The firmware must:

- acquire every configured sensor measurement at its intended sample rate on the node that owns it;
- timestamp samples against a shared, CAN-distributed monotonic time base;
- publish measurements onto the CAN bus as schema-driven records;
- write a full-fidelity, time-aligned log to microSD on the Central Controller without depending on a live link;
- expose live telemetry over **LoRa** during the race, to the driver on the cockpit dashboard, and over **USB serial / WiFi / BLE** for pit and bench use;
- report packet sequence numbers, heartbeats, per-node sensor status, calibration state, storage health, bus health, and recoverable errors;
- preserve enough session metadata to interpret a log later, including firmware/protocol versions, the node topology, and sensor configuration;
- keep acquisition, logging, and each transport decoupled so no node or link reduces the fidelity of the authoritative log.

## System architecture

```text
[FL/FR/BL/BR corner nodes]  ~~ 5 GHz WiFi ~~>  [IMU Controller] <-- chassis IMU
       (own battery)                                 |
                                                     | CAN (500 kbit/s, 11-bit ids)
    fuel level, engine speed, CVT speed --> [Powertrain]
         brake pedal, gas pedal, steering --> [Driver Input]
              brake temps x4, CVT temp --> [Temperature]
                      stress sensors 1-8 --> [Stress]
                                                     |
                                       GPS --> [Central Controller]
                                                 |-> authoritative microSD log (full fidelity, time-aligned)
                                                 |-> LoRa  -> pit (in-race critical subset)
                                                 |-> WiFi  -> cockpit dashboard (own battery) and pit laptop
                                                 |-> BLE   -> phone/tablet (walk-up diagnostics)
                                                 |-> USB   -> bench (log dump / debug / flash)
```

### Nodes

| Node | Id | Board | Signals (catalog names, rates) | Notes |
| --- | --- | --- | --- | --- |
| `imu` | 0x01 | ESP32-C5 | `IMU_CH_*` 100 Hz; forwards `IMU_FL/FR/BL/BR_*` 100 Hz; `CORNER_*` status 1 Hz | corner WiFi access point |
| `powertrain` | 0x02 | ESP32 | `ENGINE_RPM`, `CVT_RPM` 100 Hz; `FUEL_LEVEL` 1 Hz | |
| `driver_input` | 0x03 | ESP32 | `BRAKE_PEDAL`, `GAS_PEDAL`, `STEERING_ANGLE` 100 Hz | |
| `temperature` | 0x04 | ESP32 | `TEMP_BRAKE_FL/FR/BL/BR`, `TEMP_CVT` 10 Hz | |
| `stress` | 0x05 | ESP32 | `STRESS_1..8` 100 Hz | |
| `central` | 0x06 | ESP32-C5 | `GPS_*` 10 Hz; `DASH_BATTERY`, `DASH_LINK` 1 Hz | logger, time master, LoRa, dashboard link |
| `dashboard` | 0x07 | ESP32-C5 | `DASH_BATTERY` (reported to the Central) | driver's display, WiFi only |
| `corner_fl/fr/bl/br` | 0x08..0x0B | Mini ESP32-C5 | their `IMU_xx_*` 100 Hz, `CORNER_xx_BATTERY` 1 Hz | WiFi only, own battery |

Rates and frames are DAQ-WIRING.md section 7, implemented as a table
(`car/components/protocol/frames.c`); each node's plan (`car/nodes/<name>/main/<name>_plan.c`)
must match it, which the host tests check together with the bus budget. The cockpit `dashboard`
node is the driver's display in the car; it is unrelated to the `lbr-dashboard` desktop app.

Each sensor node is an independent, independently testable ESP32 that:

- owns and samples its sensors on a grid of the shared time base;
- publishes schema-driven measurement and health frames onto CAN (the corners through the IMU
  Controller);
- optionally keeps a **local microSD log** if it is a high-rate or safety-critical node (none in
  the 2026 car; see redundancy below).

Hardware-specific sensor code lives behind narrow interfaces so acquisition, serialization,
queue-pressure, and timing logic can be exercised in host tests without an ESP32 attached.

### Central Controller

The Central fills the gateway/logger role. It:

- sniffs the entire CAN bus and writes the **authoritative, time-aligned, full-fidelity** microSD log;
- is the single source of truth and single pull-point after a run;
- broadcasts the CAN time-sync frames that discipline every node's clock;
- samples its GPS and puts it on CAN like any other node;
- drives all live transports (LoRa, WiFi, BLE, USB) and the dashboard from the logical record stream.

A slow or disconnected live link must **never** reduce the fidelity of the authoritative log.
Acquisition, logging, and each transport use **bounded queues** with explicit overflow/error
counters rather than blocking one another indefinitely.

### Logging and redundancy

- The Central's microSD log is the **authoritative** record: all frames seen on the bus (including the Central's own), plus events, health, and session metadata.
- **Do not put an SD card on every node.** Local node logging is reserved for the few nodes whose native sample rate exceeds what the shared CAN bus should carry, or whose data is too critical to risk losing. The 2026 plans publish every sample on CAN, so no node logs locally.
- When a node logs locally, its log uses the same versioned record/health model as the Central's log so the two can be reconciled afterward.

## CAN bus

CAN is the backbone. Every measurement, event, and health record travels as a schema-driven
CAN frame; nodes are decoupled publishers/subscribers rather than point-to-point links.

- Transport: ESP32 **TWAI** peripheral + an external 3.3 V CAN transceiver (e.g. SN65HVD230). Classic CAN at **500 kbit/s** (DAQ-WIRING.md: robust on a vibrating car; 1 Mbit/s only if the measured load is too high). CAN FD is a documented future option.
- The id map is DAQ-WIRING.md section 7 with **11-bit standard ids**, lower id = higher priority. Several values are packed per frame; which signal sits where, in which encoding and scale, is a frame table exported to the shared catalog, so decoders are schema-driven ([`docs/can-ids.md`](docs/can-ids.md)).
- The bus is **bandwidth-limited and shared**: the frame table fixes each node's load, and the host tests check the whole bus (41.7 % of 500 kbit/s planned, ceiling 50 %).
- Bus bit rate, termination (120 ohm at the IMU Controller and the Central), and topology are verified against the final harness before bring-up.

## Time, sequence, and freshness

Every sample or batch carries a monotonic timestamp with a documented epoch and unit,
**referenced to the shared time base**, not to laptop or Central packet-arrival time.

- The Central broadcasts a **CAN time-sync frame** pair 10 times a second; nodes discipline their local monotonic clocks to it, and the IMU Controller relays the time to the corner nodes over WiFi, so every sample shares one timeline.
- Sampling is locked to a grid of that clock (sample k at k x period), so each CAN frame's 16-bit sample counter carries the sample's exact time; WiFi or CAN latency never becomes measurement time.
- Every live packet and WiFi datagram carries a monotonically increasing **sequence number** (per logical stream) so receivers detect drops, duplicates, and reordering. This matters most on LoRa and WiFi, which are lossy.
- Heartbeat/status records are sent even when measurements are unchanged, and include at minimum:
  - firmware and protocol versions, and node identity;
  - node uptime and reset/boot reason;
  - per-node sensor and (where present) SD-card health;
  - acquisition, logging, and publish rates;
  - queue high-water marks and overflow counts;
  - CAN bus health (error counters, bus-off events, dropped frames);
  - write and transport error counts;
  - calibration state when available;
  - last successful sample and last SD-write age.

Counter widths and rollover behavior are specified in the shared protocol.

## Live transports

All transports implement the **same logical record and health model** behind a transport
interface (one packet framing for every link, including the corner and dashboard WiFi links). Transport-specific framing and bandwidth policy must not change the meaning of a
logged signal. Each link occupies a distinct range/bandwidth/availability niche:

### LoRa - in-race telemetry

The only link assumed live during a race. Long range but **very low bandwidth** (and subject
to regional duty-cycle limits), so it carries a **prioritized, downsampled critical subset**:
health/heartbeat, key temperatures (e.g. engine/CVT/brake), battery/fuel, and fault/alarm
**events**. Packets are self-describing, sequence-numbered, and loss-tolerant; framing must let
a receiver resynchronize after dropped or corrupted packets.

### USB serial - wired bench/pit

Fastest, zero-setup wired link from the Central: bulk log dump after a run, bench debugging,
and flashing. Read-only telemetry from the vehicle's perspective for the initial release.

### WiFi - pit / paddock

Short-range, high-bandwidth, power-hungry. The Central's access point (5 GHz by default, 2.4 GHz
fallback) serves the pit: near-full-rate live view when the car is stationary or passing pit,
plus **OTA firmware updates** and configuration push.

### Cockpit dashboard and corner links - 5 GHz WiFi on the car

The cockpit dashboard joins the Central's access point and gets display values and node status
10 times a second over UDP; it reports its battery back. The corner nodes join the IMU
Controller's access point and stream their IMU samples (each datagram repeats the previous batch,
so one lost datagram costs nothing). Details: DAQ-WIRING.md section 8 and
[`docs/protocol.md`](docs/protocol.md).

### BLE - walk-up diagnostics

Low-power, phone/tablet range: quick status, calibration state, and configuration at the car
without a cable.

Future radio or CAN-FD adapters must implement the same logical record and health model.

## Shared protocol relationship

The dashboard and firmware must implement **one versioned protocol/log specification**. The
protocol is schema-driven: signal identifiers, value types, units, source/device identity, node
topology, sample rates, and where each value sits in a CAN frame are metadata (the catalog and
frame table), not assumptions baked into a decoder. Different signals may have different sample rates.

Firmware reports **sensor-frame** values faithfully. Per-node sensor-to-vehicle mounting
transforms are stored as session/configuration metadata and applied consistently by the
dashboard, so remounting a node does not require analysis-code changes.

The canonical specification should define:

- signal/schema descriptors and stable identifiers;
- the CAN ID / arbitration map and node identity scheme;
- scalar, vector, boolean, string/event, and status value types;
- canonical units and display metadata;
- device/session/topology metadata;
- the shared time base, monotonic timestamps, sequence numbers, rollover, and ordering;
- record, CAN-frame, and transport framing, checksums/integrity checks, and recovery;
- heartbeat and health fields (including bus health);
- compatibility rules for older logs and unknown future signals/nodes;
- golden binary/log fixtures used by both repositories.

The initial canonical specification may live in `lbr-dashboard`, with generated/copied bindings
or golden fixtures consumed here. Do not let independent, incompatible definitions evolve in the
two codebases, or across nodes.

## Hardware and pin mapping

The 2026 board set is **ESP32-based and still being finalized**. Per-node GPIO assignments must
come from each node's schematic and are **TBD** until the electronics team confirms them. Do not
hard-code pin numbers or assign line semantics in code, docs, or the protocol until confirmed;
use an explicit unconfigured/TBD placeholder if a build needs one.

Each node will minimally need:

| Function | Interface | Notes |
| --- | --- | --- |
| CAN TX / RX | TWAI | To external CAN transceiver; GPIO **TBD** per node |
| Sensor bus(es) | I2C / SPI / analog | Depends on the node's sensors; **TBD** |
| microSD (Central, and local-logging nodes) | SPI / SDMMC | **TBD** |
| LoRa radio (Central) | SPI (+ control lines) | e.g. SX127x/RFM95 class module; **TBD** |
| GPS receiver (Central) | UART, NMEA 0183 | any module; **TBD** |
| WiFi (IMU, Central, corners, dashboard) | ESP32-C5 radio, 5 GHz | 2.4 GHz fallback by menuconfig |

Electrical voltage levels, transceiver choice, pull-ups, bus termination, card-detect,
interrupt, reset, and power-control details must be verified against the final schematic and
harness before any hardware-initialization code is merged.

## Repository layout

```text
car/                          # ON THE CAR: everything that is flashed to an ESP32
  cmake/                      #   warnings-as-errors policy, ESP-IDF version pin check
  components/                 #   shared libraries (ESP-IDF components); *_esp.c = hardware side
    util/                     #     byte order, CRC-32, COBS, bounded ring buffer
    protocol/                 #     signal catalog (signals.def), CAN frame table (frames.c), node ids
    health/                   #     heartbeat record + page codec, reset reason
    can/                      #     CAN id map, field codec, sync-tracking decoder; TWAI adapter
    timebase/                 #     shared time sync servo
    logging/                  #     recoverable log format, bounded queue, SD service
    sensors/                  #     driver interface, configurable inputs, NMEA GPS
    node/                     #     plans/validation, grid-locked acquisition, sensor-node runtime
    transport/                #     packet framing, LoRa policy
    links/                    #     the Central's USB/LoRa/WiFi/BLE links
    wlink/                    #     corner and dashboard WiFi links (cores + C5 WiFi/UDP glue)
    gateway/                  #     the Central's logic, runtime config, status text
  nodes/                      #   one ESP-IDF app per physical node (DAQ-WIRING.md)
    sdkconfig.common          #     settings shared by every node
    central/                  #     CAN logger, time master, GPS, LoRa, dashboard link (ESP32-C5)
    imu/                      #     chassis IMU + corner receiver (ESP32-C5)
    powertrain/ driver_input/ temperature/ stress/   # wired sensor controllers (ESP32)
    dashboard/                #     cockpit display over WiFi (ESP32-C5)
    corner/                   #     one firmware for the four corner nodes (Mini ESP32-C5)
dev/                          # DEVELOPMENT ONLY: never built into firmware
  CMakeLists.txt              #   host build: unit tests, fixtures, simulator
  sim/                        #   car simulator: real firmware logic on a simulated car + browser UI
  tests/host/                 #   unit tests of the pure logic + fixture generator
  tests/fixtures/             #   golden fixtures shared with lbr-dashboard
  ci/                         #   CI-only sdkconfig overlays (not pin maps)
  tools/lbr_decode.py         #   reference decoder: logs, streams, USB serial, fixture self-test
docs/                         # protocol, CAN ids, hardware, sensors
.idf-version                  # the one ESP-IDF release every build must use
.github/workflows/ci.yml
```

**`car/` is what goes on the car; `dev/` never does.** Node builds only see `car/`, CI fails if
anything in `car/` includes or builds from `dev/`, and the bench sensor simulator
(`CONFIG_LBR_SENSOR_SIM`) is compiled into a node only when that bench option is on.

The node runtime, the Central, the corner and dashboard links, and the SD session keeper are pure
logic (`node_core.c`, `gw_core.c`, `corner_core.c`, `corner_rx.c`, `dash_core.c`, `log_keeper.c`)
behind small hardware tables; the `*_esp.c` files only connect
them to FreeRTOS, TWAI, SD, and the radios. That is what lets the host tests and the
simulator run exactly the code that is flashed.

Each node in `car/nodes/` is a self-contained ESP-IDF project (its own top-level `CMakeLists.txt`
+ `main/`) that pulls in the shared `car/components/` via `EXTRA_COMPONENT_DIRS` and builds only
what its `main` needs. Node apps stay thin: a plan (pure data: node id, channels, rates) and
driver binding. Shared logic lives in `car/components/` behind narrow interfaces so protocol,
queue, time-sync, and log-recovery behavior is exercised in host tests without hardware.
Adding a node means copying a `car/nodes/<name>/` directory, giving it a new id in
`car/components/protocol/include/lbr/nodes.h`, its frames in `frames.c`, and adding its plan to
`dev/tests/host/CMakeLists.txt`.

## Getting started

Everything below runs from the repository root unless it says otherwise. Steps 1 to 3 need no
ESP32 and no ESP-IDF; step 4 builds and flashes the firmware.

### 1. Prerequisites

| For | You need |
| --- | --- |
| tests and simulator (`dev/`) | CMake 3.16+, Ninja, a C11 compiler (gcc or clang), Python 3.8+ |
| firmware (`car/`) | ESP-IDF, **exactly the release in [`.idf-version`](.idf-version)** (currently **v6.1**) |

On Windows, use WSL or MSYS2 (MinGW-w64) for the C build; the simulator UI only needs Python.
Debian/Ubuntu: `sudo apt install cmake ninja-build gcc python3`.

ESP-IDF, with Espressif's installer (EIM) or by hand:

```sh
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v6.1
~/esp/esp-idf-v6.1/install.sh esp32
. ~/esp/esp-idf-v6.1/export.sh        # in every new shell before using idf.py
```

Every node build checks the active IDF against `.idf-version` (`car/cmake/idf_version.cmake`)
and stops on any other release, or on an IDF git checkout that is not exactly on that tag, so
every developer and CI build the same firmware. Moving to a new release is a deliberate change:
update `.idf-version` in its own PR (CI reads its `espressif/idf` container tag from the same
file) and rebuild every node.

### 2. Build and run the tests (no hardware)

```sh
cmake -S dev -B build-dev -G Ninja
cmake --build build-dev
ctest --test-dir build-dev --output-on-failure   # unit tests, golden fixtures, simulator scenarios
ctest --test-dir build-dev -R logging            # one suite (or e.g. -R sim_can_faults)
python3 dev/tools/lbr_decode.py --selftest       # independent decoder vs the golden fixtures
```

The host build uses ASan and UBSan by default (`-DLBR_SANITIZE=OFF` to turn them off).
Regenerate the golden fixtures only for an intended format change:
`cmake --build build-dev --target fixtures`.

### 3. Run the simulator

The simulator ([`dev/sim/README.md`](dev/sim/README.md)) runs every node's real firmware logic on
a simulated car. After step 2:

```sh
python3 dev/sim/ui/server.py                     # then open http://127.0.0.1:8765/
python3 dev/sim/ui/server.py --port 9000 --seed 7 --out /tmp/sim-run   # other port, seed, output
```

In the browser:

- the car from above: the eleven nodes where DAQ-WIRING puts them, every sensor where it sits,
  the daisy-chained CAN trunk with frames moving along it, and the 5 GHz WiFi links; colors show
  state (green ok, amber degraded or no data, red fault or lost, grey not fitted or off);
- **click a node** for its state, time sync, CAN controller or WiFi link, heartbeat, slots, SD
  card, and links, with buttons to unplug its CAN cable, force bus-off, take its WiFi out of
  range or add loss, reboot, power it off, change its clock drift, unplug a part, or pull the
  Central's SD card; **click a sensor** to override its value or open the
  circuit; **click the bus** to add noise;
- **Trace commands** on a node, then the **Commands** tab, shows every hardware call its firmware
  makes (CAN sends, ADC/GPIO/UART reads, SD writes, time-sync updates, link packets);
- **Events** lists every event and fault on the bus and scenario results; **Cockpit** shows the
  driver's display; **Console** takes any simulator command (`help` lists them);
- the header has pause, step 100 ms, speed (0.05x to 20x), race or pit driving, seed and
  restart, and **Play scenario**.

Without the browser, run a scenario or a timed run directly:

```sh
build-dev/sim/lbr_sim dev/sim/scenarios/sensor_faults.sim                    # exit status = failed checks
build-dev/sim/lbr_sim --trace corner_fl dev/sim/scenarios/wifi.sim          # with a node's commands
build-dev/sim/lbr_sim --seed 7 --duration 30                                 # run 30 s, print the state
```

The simulated Central writes real files to `sim-out/` (or `--out`): SD session logs
(`00000001.LBR`, ...) and the USB stream. Read them like the car's:

```sh
python3 dev/tools/lbr_decode.py log sim-out/00000001.LBR
python3 dev/tools/lbr_decode.py stream sim-out/usb_stream.bin
```

Scenarios are plain text (`at <seconds> <command>`, `expect ...` checks, `end <seconds>`) in
`dev/sim/scenarios/`; every file there runs in `ctest` and in CI. Add one for each behavior you
change. The command and check reference is in [`dev/sim/README.md`](dev/sim/README.md).

### 4. Build and flash the firmware

Each node is its own ESP-IDF project in `car/nodes/<node>` (`central`, `imu`, `powertrain`,
`driver_input`, `temperature`, `stress`, `dashboard`, `corner`). Its chip (ESP32 or ESP32-C5)
is set in its `sdkconfig.defaults`. With ESP-IDF exported:

```sh
cd car/nodes/central
idf.py build                             # first build picks the node's chip
idf.py -p /dev/ttyUSB0 flash monitor     # COM3 on Windows; Ctrl+] leaves the monitor
idf.py menuconfig                        # LBR menus: pins, CAN, SD, inputs, links, bench options
```

Build every node from the repository root (warnings are errors, as in CI):

```sh
for n in central imu powertrain driver_input temperature stress dashboard corner; do
  (cd car/nodes/$n && idf.py build) || break
done
```

- **Pins** are Kconfig settings that default to `-1` until the schematics confirm them
  (`docs/hardware.md`); the firmware refuses to drive an unset pin and reports it. Set them in
  `menuconfig` per node.
- **Sensors** on analog, pulse, or switch inputs and an NMEA GPS are wired and calibrated in
  `menuconfig` with no code (`docs/sensors.md`).
- **Bench only:** `CONFIG_LBR_SENSOR_SIM=y` feeds simulated sensor data (flagged in every
  heartbeat and log; never race with it). WiFi stays off until its password is set locally:
  `CONFIG_LBR_WIFI_PASSWORD` on the Central and the dashboard, `CONFIG_LBR_CORNER_PASSWORD` on
  the IMU Controller and the corners; never commit them.
- **Corner position:** flash the `corner` firmware four times with `CONFIG_LBR_CORNER_POSITION`
  set to FL, FR, BL, and BR.
- **Link check** (what CI also builds; placeholder pins, not a pin map):
  `idf.py -DLBR_SDKCONFIG_EXTRA=$PWD/../../../dev/ci/sdkconfig.linkcheck build` from a node directory.

### 5. Bench tools (real Central)

```sh
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0                     # live view over USB (pip install pyserial)
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd LIST          # session files on the SD card
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd "DUMP 00000012.LBR" --out run.lbr
python3 dev/tools/lbr_decode.py log run.lbr                             # decode a session log
```

### Troubleshooting

| Symptom | Fix |
| --- | --- |
| `LBR: this project requires ESP-IDF v6.1 ...` | export the IDF from `.idf-version` (`. ~/esp/esp-idf-v6.1/export.sh`) |
| `... is not checked out at the release tag` | `git -C "$IDF_PATH" checkout v6.1 && git -C "$IDF_PATH" submodule update --init --recursive` |
| `lbr_sim not found` from the UI | build step 2 first, or pass `--sim path/to/lbr_sim` |
| UI port already in use | `python3 dev/sim/ui/server.py --port 9000` |
| a node logs `CAN_PINS_UNSET` / `configuration rejected` | set its pins in `menuconfig`; the log line names the bad setting |

## Test workflow

Before merging firmware changes:

1. run host-side unit tests for protocol encoding/decoding, CAN record (de)serialization, timestamp/sequence rollover, time-base sync, queues, log recovery, and configuration validation;
   and the simulator scenarios, adding one for any behavior the change touches;
2. build every affected node with **warnings treated as errors**;
3. run formatting and static-analysis checks;
4. replay golden fixtures through the dashboard compatibility tests;
5. complete hardware-in-the-loop smoke tests - multi-node CAN acquisition, time sync (CAN and WiFi), Central SD logging, corner and dashboard WiFi links, LoRa/USB/WiFi/BLE framing, node disconnect/reconnect, bus-off recovery, and fault reporting - when hardware behavior changes.

CI (`.github/workflows/ci.yml`) runs all host tests and simulator scenarios with gcc and clang,
formatting, static analysis, the fixture replay, and builds every node on each pull request. Hardware-in-the-loop
tests may run on a dedicated runner or be recorded as a required release checklist when hardware
is unavailable to CI.

## Reliability rules

- Never silently discard samples or frames. Count and report every queue overflow, dropped CAN frame, or rejected record.
- Prefer continuing full-fidelity SD logging when a live link (LoRa/USB/WiFi/BLE) is absent or slow.
- Keep the Central's log authoritative; local node logs are redundancy, not substitutes for a shared time base.
- Start a session only after configuration is validated; persist the exact configuration and node topology with the log.
- Represent unavailable measurements and calibration explicitly rather than substituting zero.
- Make watchdog resets, brownout/boot reasons, and CAN bus-off events observable in the next session.
- Keep protocol compatibility tests synchronized with `lbr-dashboard`, and keep every node on one protocol/CAN-ID version.
