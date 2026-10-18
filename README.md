# LBR Vehicle Firmware

Embedded telemetry and data-logging firmware for Longhorn Baja Racing vehicles.

The 2026 architecture is a **distributed CAN network of ESP32 nodes**. Multiple ESP32
modules each own a different set of sensors and publish their measurements onto a shared
**CAN bus**. A central **gateway/logger node** records the authoritative log and drives the
live telemetry links. The firmware is the vehicle-side half of the telemetry system; the
companion desktop application lives in
[`longhorn-baja-racing/lbr-dashboard`](https://github.com/longhorn-baja-racing/lbr-dashboard).

The recommended runtime is **C/C++ on ESP-IDF** (Espressif's official framework), using the
ESP32's built-in **TWAI** controller for CAN with an external CAN transceiver.

The 2026 vehicle carries seven sensor projects, each its own CAN node: **IMU/GPS, suspension,
driver inputs, temperature, fuel gauge, stress (strain), and the cockpit dashboard**, plus the
gateway/logger.

## Project status

The framework is in place and tested; sensor parts and pins are not chosen yet.

- **Shared libraries** (`car/components/`): protocol + signal catalog, CAN ID map and TWAI
  adapter, CAN-distributed time sync, recoverable SD logging, heartbeat/health, sensor
  framework, node runtime, live transports (USB, LoRa, WiFi, BLE), and gateway logic. The
  hardware-independent parts run in host tests under ASan/UBSan.
- **Node apps** (`car/nodes/`): the gateway plus one app per sensor project. Every sensor node
  publishes its full channel list today; channels without a chosen part report
  `valid = false / NOT_FITTED` until a driver is bound (or simulated data with
  `CONFIG_LBR_SENSOR_SIM`, bench only). Analog, pulse, and switch sensors and any NMEA GPS
  need no driver code: they are wired and calibrated from menuconfig
  ([`docs/sensors.md`](docs/sensors.md)).
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
- write a full-fidelity, time-aligned log to microSD on the gateway node without depending on a live link;
- expose live telemetry over **LoRa** during the race and over **USB serial / WiFi / BLE** for pit and bench use;
- report packet sequence numbers, heartbeats, per-node sensor status, calibration state, storage health, bus health, and recoverable errors;
- preserve enough session metadata to interpret a log later, including firmware/protocol versions, the node topology, and sensor configuration;
- keep acquisition, logging, and each transport decoupled so no node or link reduces the fidelity of the authoritative log.

## System architecture

```text
[sensor node A]  [sensor node B]  [sensor node C]  ...        (each: ESP-IDF + TWAI)
   sensors          sensors          sensors
      |                |                |
      +--------- shared CAN bus (TWAI + transceiver) ----------+
                           |
                   [gateway / logger node]
                     |-> authoritative microSD log (full fidelity, time-aligned)
                     |-> LoRa  -> dashboard (in-race critical subset)
                     |-> WiFi  -> dashboard (pit: bulk / OTA / config)
                     |-> BLE   -> phone/tablet (walk-up diagnostics)
                     |-> USB   -> bench (log dump / debug / flash)
```

### Nodes

| Node | Id | Signals (catalog names) | Notes |
| --- | --- | --- | --- |
| `gateway` | 0x01 | events/faults, its own heartbeat | logger, time master, telemetry |
| `imu_gps` | 0x02 | `ACCEL_*`, `GYRO_*` 100 Hz; `ORIENT_QUAT_*` 20 Hz; `MAG_*` 10 Hz; `GPS_*` 10 Hz | |
| `suspension` | 0x03 | `SUSP_TRAVEL_FL/FR/RL/RR` 200 Hz | |
| `driver_inputs` | 0x04 | `THROTTLE_POS`, `BRAKE_PRESS_FRONT/REAR`, `STEERING_ANGLE` 100 Hz; `BRAKE_SWITCH` 50 Hz | |
| `temperature` | 0x05 | engine, CVT, gearbox, brakes 10 Hz; ambient 1 Hz; 4 aux 2 Hz | |
| `fuel_gauge` | 0x06 | `FUEL_LEVEL`, `FUEL_VOLUME` 5 Hz | |
| `stress` | 0x07 | `STRAIN_1..8` 1 kHz | local SD log at full rate; CAN gets 100 Hz |
| `dashboard` | 0x08 | `DASH_BUTTONS` 20 Hz | in-cockpit display; listens to the whole bus |

Rates are starting points in each node's plan (`car/nodes/<name>/main/<name>_plan.c`). The host
tests validate every plan and the system-wide CAN load whenever one changes. The cockpit
`dashboard` node is the driver's display in the car; it is unrelated to the `lbr-dashboard`
desktop app.

Each sensor node is an independent, independently testable ESP32 that:

- owns and samples its sensors at their configured rates;
- timestamps samples against the shared time base;
- publishes schema-driven measurement and health records onto CAN;
- optionally keeps a **local microSD log** if it is a high-rate or safety-critical node (see redundancy below).

Hardware-specific sensor code lives behind narrow interfaces so acquisition, serialization,
queue-pressure, and timing logic can be exercised in host tests without an ESP32 attached.

### Gateway / logger node

One node is the gateway. It:

- sniffs the entire CAN bus and writes the **authoritative, time-aligned, full-fidelity** microSD log;
- is the single source of truth and single pull-point after a run;
- broadcasts the CAN time-sync frame that disciplines every node's clock;
- drives all live transports (LoRa, WiFi, BLE, USB) from the logical record stream.

A slow or disconnected live link must **never** reduce the fidelity of the authoritative log.
Acquisition, logging, and each transport use **bounded queues** with explicit overflow/error
counters rather than blocking one another indefinitely.

### Logging and redundancy

- The gateway's microSD log is the **authoritative** record: all records seen on the bus, plus events, health, and session metadata.
- **Do not put an SD card on every node.** Local node logging is reserved for the few nodes whose native sample rate exceeds what the shared CAN bus should carry, or whose data is too critical to risk losing. This spends redundancy where it matters instead of multiplying weight, cost, and SD-corruption surface across the car.
- When a node logs locally, its log uses the same versioned record/health model as the gateway log so the two can be reconciled afterward.

## CAN bus

CAN is the backbone. Every measurement, event, and health record travels as a schema-driven
CAN frame; nodes are decoupled publishers/subscribers rather than point-to-point links.

- Transport: ESP32 **TWAI** peripheral + an external CAN transceiver (e.g. SN65HVD230 / TJA1051). Classic CAN at 1 Mbit/s (provisional, #15) is the baseline; CAN FD is a documented future option if bandwidth gets tight.
- The bus is **bandwidth-limited and shared**: high-rate data may be logged locally at full fidelity while a reduced version is published on CAN. CAN fidelity is a budget, not an assumption: every node configuration is checked against a per-node budget at boot, and the host tests check the whole system (48.7% of 1 Mbit/s today, ceiling 60%).
- The CAN ID plan (`docs/can-ids.md`) uses 29-bit IDs carrying priority, kind, source node, and signal id. Faults and time sync have the highest priorities.
- Bus bit rate, termination (120 ohm at both physical ends), and topology are verified against the final harness before bring-up.

## Time, sequence, and freshness

Every sample or batch carries a monotonic timestamp with a documented epoch and unit,
**referenced to the shared time base**, not to laptop or gateway packet-arrival time.

- The gateway broadcasts a periodic **CAN time-sync frame**; nodes discipline/offset their local monotonic clocks to it so all logs - gateway and any local node logs - share one timeline.
- Every live packet carries a monotonically increasing **sequence number** (per logical stream) so the dashboard can detect drops, duplicates, and reordering. This matters most on LoRa, which is lossy.
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
interface. Transport-specific framing and bandwidth policy must not change the meaning of a
logged signal. Each link occupies a distinct range/bandwidth/availability niche:

### LoRa - in-race telemetry

The only link assumed live during a race. Long range but **very low bandwidth** (and subject
to regional duty-cycle limits), so it carries a **prioritized, downsampled critical subset**:
health/heartbeat, key temperatures (e.g. engine/CVT/brake), battery/fuel, and fault/alarm
**events**. Packets are self-describing, sequence-numbered, and loss-tolerant; framing must let
a receiver resynchronize after dropped or corrupted packets.

### USB serial - wired bench/pit

Fastest, zero-setup wired link from the gateway: bulk log dump after a run, bench debugging,
and flashing. Read-only telemetry from the vehicle's perspective for the initial release.

### WiFi - pit / paddock

Short-range, high-bandwidth, power-hungry - pit use only. Near-full-rate live view when the car
is stationary or passing pit, plus **OTA firmware updates** and configuration push.

### BLE - walk-up diagnostics

Low-power, phone/tablet range: quick status, calibration state, and configuration at the car
without a cable.

Future radio or CAN-FD adapters must implement the same logical record and health model.

## Shared protocol relationship

The dashboard and firmware must implement **one versioned protocol/log specification**. The
protocol is schema-driven: signal identifiers, value types, units, source/device identity, node
topology, and sample rates are metadata, not assumptions baked into a fixed packet or CAN-frame
layout. Different signals may have different sample rates.

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
| microSD (gateway, and local-logging nodes) | SPI / SDMMC | **TBD** |
| LoRa radio (gateway) | SPI (+ control lines) | e.g. SX127x/RFM95 class module; **TBD** |

Electrical voltage levels, transceiver choice, pull-ups, bus termination, card-detect,
interrupt, reset, and power-control details must be verified against the final schematic and
harness before any hardware-initialization code is merged.

## Repository layout

```text
car/                          # ON THE CAR: everything that is flashed to an ESP32
  cmake/                      #   warnings-as-errors policy, ESP-IDF version pin check
  components/                 #   shared libraries (ESP-IDF components); *_esp.c = hardware side
    util/                     #     byte order, CRC-32, COBS, bounded ring buffer
    protocol/                 #     signal catalog (signals.def), sample codec, node ids
    health/                   #     heartbeat record + CAN page codec, reset reason
    can/                      #     CAN ID map + frame codec; TWAI adapter
    timebase/                 #     CAN-distributed time sync servo
    logging/                  #     recoverable log format, bounded queue, SD service
    sensors/                  #     driver interface, configurable inputs, NMEA GPS
    node/                     #     node plans/validation, shared sensor-node runtime
    transport/                #     packet framing, LoRa policy; USB/LoRa/WiFi/BLE links
    gateway/                  #     gateway runtime config + status text
  nodes/                      #   one ESP-IDF app per physical node
    sdkconfig.common          #     settings shared by every node
    gateway/                  #     CAN logger + telemetry + time-sync source
    imu_gps/ suspension/ driver_inputs/ temperature/ fuel_gauge/ stress/ dashboard/
dev/                          # DEVELOPMENT ONLY: never built into firmware
  tests/host/                 #   CMake/CTest build of the pure logic + fixture generator
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

Each node in `car/nodes/` is a self-contained ESP-IDF project (its own top-level `CMakeLists.txt`
+ `main/`) that pulls in the shared `car/components/` via `EXTRA_COMPONENT_DIRS` and builds only
what its `main` needs. Node apps stay thin: a plan (pure data: node id, channels, rates) and
driver binding. Shared logic lives in `car/components/` behind narrow interfaces so protocol,
queue, time-sync, and log-recovery behavior is exercised in host tests without hardware.
Adding a node means copying a `car/nodes/<name>/` directory, giving it a new id in
`car/components/protocol/include/lbr/nodes.h`, and adding its plan to `dev/tests/host/CMakeLists.txt`.

## Build workflow

Prerequisites: ESP-IDF **exactly the release in [`.idf-version`](.idf-version)** (currently
**v6.1**) and its toolchain, with its export script sourced so `idf.py` is on your `PATH`.
Every node build checks this (`car/cmake/idf_version.cmake`) and stops with an error on any other
version, or on a git checkout of IDF that is not sitting on that release tag, so every
developer and CI produce firmware from the same IDF. Install it with Espressif's installer
(EIM) or:

```sh
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v6.1
~/esp/esp-idf-v6.1/install.sh esp32
. ~/esp/esp-idf-v6.1/export.sh
```

Moving to a new IDF release is a deliberate change: update `.idf-version` in its own PR (CI
takes its `espressif/idf` container tag from the same file) and rebuild every node.

```sh
# from a node app directory, e.g. car/nodes/gateway
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
idf.py menuconfig          # LBR menus: CAN, microSD, node runtime, transports
```

Pins live in Kconfig (`CONFIG_LBR_CAN_*_GPIO`, `CONFIG_LBR_SD_*`, `CONFIG_LBR_LORA_*`) and
default to `-1` until the schematic confirms them (`docs/hardware.md`). Bench options:
`CONFIG_LBR_SENSOR_SIM=y` (simulated sensor data, flagged in every heartbeat and log) and,
on the gateway, `CONFIG_LBR_WIFI_PASSWORD` (WiFi stays off until it is set; never commit it).

An extra sdkconfig overlay can be layered on any node:

```sh
idf.py -DLBR_SDKCONFIG_EXTRA=$PWD/../../../dev/ci/sdkconfig.linkcheck build
```

Host-side unit tests build and run without hardware:

```sh
cmake -S dev/tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
ctest --test-dir build-host -R <test_name>        # e.g. -R logging
cmake --build build-host --target fixtures       # regenerate golden fixtures (format change)
python3 dev/tools/lbr_decode.py --selftest           # reference decoder vs fixtures
```

Bench tools:

```sh
python3 dev/tools/lbr_decode.py log 00000012.LBR             # decode a session log
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0          # live gateway view over USB
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd LIST
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd "DUMP 00000012.LBR" --out run.lbr
```

The node projects are `lbr_gateway`, `lbr_imu_gps`, `lbr_suspension`, `lbr_driver_inputs`,
`lbr_temperature`, `lbr_fuel_gauge`, `lbr_stress`, and `lbr_dashboard`. CI builds them in the
`espressif/idf` image whose tag is read from `.idf-version`.

## Test workflow

Before merging firmware changes:

1. run host-side unit tests for protocol encoding/decoding, CAN record (de)serialization, timestamp/sequence rollover, time-base sync, queues, log recovery, and configuration validation;
2. build every affected node with **warnings treated as errors**;
3. run formatting and static-analysis checks;
4. replay golden fixtures through the dashboard compatibility tests;
5. complete hardware-in-the-loop smoke tests - multi-node CAN acquisition, time sync, gateway SD logging, LoRa/USB/WiFi/BLE framing, node disconnect/reconnect, bus-off recovery, and fault reporting - when hardware behavior changes.

CI (`.github/workflows/ci.yml`) runs all host tests with gcc and clang, formatting, static
analysis, the fixture replay, and builds every node on each pull request. Hardware-in-the-loop
tests may run on a dedicated runner or be recorded as a required release checklist when hardware
is unavailable to CI.

## Reliability rules

- Never silently discard samples or frames. Count and report every queue overflow, dropped CAN frame, or rejected record.
- Prefer continuing full-fidelity SD logging when a live link (LoRa/USB/WiFi/BLE) is absent or slow.
- Keep the gateway log authoritative; local node logs are redundancy, not substitutes for a shared time base.
- Start a session only after configuration is validated; persist the exact configuration and node topology with the log.
- Represent unavailable measurements and calibration explicitly rather than substituting zero.
- Make watchdog resets, brownout/boot reasons, and CAN bus-off events observable in the next session.
- Keep protocol compatibility tests synchronized with `lbr-dashboard`, and keep every node on one protocol/CAN-ID version.
