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

There is **no GPS requirement for the 2026 system**.

## Project status

The repository skeleton is in place and the host test build is green:

- shared libraries scaffolded in `components/` (protocol with a stable signal catalog,
  transport interface, logging, health, timebase, generic sensor interface);
- one ESP-IDF app per node in `nodes/` (`gateway`, `powertrain`, `dynamics`);
- a `tests/host/` CMake/CTest build that compiles the pure logic with warnings as errors.

Firmware logic is still stubbed (`TODO`s), and the following are deliberately **TBD** pending
decisions: concrete sensor parts, per-node GPIO/pins, the frozen wire/log byte layout, and the
exact CAN ID map. See `docs/` for the per-topic trackers.

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

- Transport: ESP32 **TWAI** peripheral + an external CAN transceiver (e.g. SN65HVD230 / TJA1051). Classic CAN (up to 1 Mbit/s, shared across all nodes) is the baseline; CAN FD is a documented future option if bandwidth gets tight.
- The bus is **bandwidth-limited and shared**: high-rate data may be logged locally at full fidelity while a reduced version is published on CAN. CAN fidelity is a budget, not an assumption.
- Define a stable, documented CAN ID / arbitration plan (message IDs, priorities, source node identity). Safety/alarm and time-sync frames get priority IDs.
- Specify bus bit rate, termination (120 ohm at both physical ends), and topology against the final harness before bring-up.

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
cmake/warnings.cmake      # shared warnings-as-errors policy
docs/
  hardware.md             # node grouping + pin map (TBD)
  protocol.md             # firmware-side view of the shared protocol
  can-ids.md              # CAN ID / arbitration map (draft)
components/               # shared, host-testable libraries (ESP-IDF components)
  protocol/               #   signal catalog (signals.h) + sample encode/decode
  transport/              #   one record model behind LoRa/USB/WiFi/BLE
  logging/                #   append-oriented, recoverable SD log
  health/                 #   heartbeat/status record
  timebase/               #   shared CAN-distributed monotonic clock
  sensors/                #   generic sensor interface (concrete parts TBD)
nodes/                    # one standalone ESP-IDF app per physical node
  gateway/                #   CAN logger + telemetry + time-sync source
  powertrain/             #   engine RPM, engine/CVT temps, fuel level
  dynamics/               #   IMU, vehicle/wheel speed
tests/
  host/                   #   plain-CMake/CTest build of the pure logic
  fixtures/               #   golden fixtures shared with lbr-dashboard
tools/
.github/workflows/
```

Each node in `nodes/` is a self-contained ESP-IDF project (its own top-level `CMakeLists.txt`
+ `main/`) that pulls in the shared `components/` via `EXTRA_COMPONENT_DIRS`. Node apps stay
thin - wiring, configuration, and their sensor set. Shared logic lives in `components/` behind
narrow interfaces so protocol, queue, time-sync, and log-recovery behavior is exercised in host
tests without hardware. The node grouping above is a starting point (grouped by vehicle zone to
shorten the harness); adding a node means copying a `nodes/<name>/` directory.

## Build workflow

Prerequisites: install **ESP-IDF** (pin a known version) and its toolchain, and run its export
script so `idf.py` is on your `PATH`.

```sh
# from a node app directory, e.g. nodes/gateway
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

Host-side unit tests build and run without hardware (e.g. with CMake/CTest against the
`components/` libraries):

```sh
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
ctest --test-dir build-host -R <test_name>   # run a single test
```

The node projects are `lbr_gateway`, `lbr_powertrain`, and `lbr_dynamics` (one per directory
under `nodes/`). Production/release builds pin an ESP-IDF version and a reproducible toolchain
in CI.

## Test workflow

Before merging firmware changes:

1. run host-side unit tests for protocol encoding/decoding, CAN record (de)serialization, timestamp/sequence rollover, time-base sync, queues, log recovery, and configuration validation;
2. build every affected node with **warnings treated as errors**;
3. run formatting and static-analysis checks;
4. replay golden fixtures through the dashboard compatibility tests;
5. complete hardware-in-the-loop smoke tests - multi-node CAN acquisition, time sync, gateway SD logging, LoRa/USB/WiFi/BLE framing, node disconnect/reconnect, bus-off recovery, and fault reporting - when hardware behavior changes.

CI runs all host tests and compiles every node on each pull request. Hardware-in-the-loop tests
may run on a dedicated runner or be recorded as a required release checklist when hardware is
unavailable to CI.

## Reliability rules

- Never silently discard samples or frames. Count and report every queue overflow, dropped CAN frame, or rejected record.
- Prefer continuing full-fidelity SD logging when a live link (LoRa/USB/WiFi/BLE) is absent or slow.
- Keep the gateway log authoritative; local node logs are redundancy, not substitutes for a shared time base.
- Start a session only after configuration is validated; persist the exact configuration and node topology with the log.
- Represent unavailable measurements and calibration explicitly rather than substituting zero.
- Make watchdog resets, brownout/boot reasons, and CAN bus-off events observable in the next session.
- Keep protocol compatibility tests synchronized with `lbr-dashboard`, and keep every node on one protocol/CAN-ID version.
