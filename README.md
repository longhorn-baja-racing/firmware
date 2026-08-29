# LBR Vehicle Firmware

Embedded telemetry and data-logging firmware for Longhorn Baja Racing vehicles.

The 2026 baseline target is a **Raspberry Pi Pico**, an **Adafruit BNO055 IMU**, and a **microSD card**. The firmware is the vehicle-side half of the telemetry system; the companion desktop application lives in [`longhorn-baja-racing/lbr-dashboard`](https://github.com/longhorn-baja-racing/lbr-dashboard).

## 2026 goals

The firmware must:

- acquire every configured BNO055 measurement at its intended sample rate;
- timestamp samples from the device's monotonic clock;
- write full-fidelity data to microSD without depending on a live connection;
- expose read-only live telemetry over USB serial for bench and vehicle testing;
- report packet sequence numbers, heartbeats, sensor status, calibration state, storage health, and recoverable errors;
- preserve enough session metadata to interpret a log later, including firmware/protocol versions and sensor configuration;
- remain extensible to future LoRa/radio and CAN transports without coupling acquisition or logging to one transport.

There is **no GPS requirement for the 2026 system**.

## Recommended implementation

Use **C/C++ with the official Raspberry Pi Pico SDK**. This gives the logger direct control over sampling, buffering, binary serialization, SD-card writes, timing, and failure recovery. MicroPython or CircuitPython may be useful for isolated hardware experiments, but they are not the recommended production runtime for the full-rate vehicle logger.

Keep the system as independently testable modules:

```text
BNO055 driver -> sample acquisition -> timestamped signal records
                                      |-> bounded SD writer queue -> microSD
                                      |-> transport queue -> USB serial
                                                           -> future LoRa/CAN
health monitor -> heartbeat/status records ---------------^
```

A slow or disconnected live link must never reduce the fidelity of the authoritative SD log. Acquisition, storage, and transport should use bounded queues with explicit overflow/error counters rather than blocking each other indefinitely.

## Hardware and pin mapping

The following mapping comes from the current schematic.

| Function | Pico GPIO | Direction / bus |
| --- | ---: | --- |
| BNO055 SDA | GP0 | I2C data |
| BNO055 SCL | GP1 | I2C clock |
| microSD SCK | GP2 | SPI clock |
| microSD MOSI | GP3 | SPI controller-out / card-in |
| microSD MISO | GP4 | SPI controller-in / card-out |
| microSD CS | GP5 | SPI chip select |
| IMU-related line | GP9 | **TBD — purpose must be confirmed from the hardware design before implementation** |

Do not assign semantics to GP9 in code, documentation, or the wire protocol until the electronics team confirms the line's purpose. Define it as an unconfigured/TBD board input only if a build needs a placeholder.

Electrical voltage, pull-up, card-detect, interrupt, reset, and power-control details must be verified against the final schematic and board before hardware initialization is merged.

## Signals for 2026

When enabled and available from the configured BNO055 operating mode, record:

- acceleration X/Y/Z;
- gyroscope X/Y/Z;
- temperature;
- quaternion/orientation;
- BNO055 calibration state;
- sensor status and self-test/error information.

The protocol is schema-driven: signal identifiers, value types, units, source/device identity, and sample rates are metadata rather than assumptions embedded in a fixed packet layout. Different signals may have different sample rates.

The firmware reports sensor-frame values faithfully. A configurable sensor-to-vehicle transform is stored as session/configuration metadata and applied consistently by the dashboard so mounting changes do not require analysis code changes.

## Time, sequence, and freshness

Every sample or sample batch must carry a device-side monotonic timestamp with a documented epoch and unit. Do not use laptop packet-arrival time as the measurement time.

Every live packet must carry a monotonically increasing sequence number so the dashboard can detect drops, duplicates, and out-of-order delivery. Heartbeat/status records must be sent even when measurements are unchanged and include, at minimum:

- firmware and protocol versions;
- device uptime;
- sensor and SD-card health;
- acquisition and logging rates;
- queue high-water marks and overflow counts;
- write and transport error counts;
- calibration state when available;
- last successful sample and SD-write age.

Counter widths and rollover behavior must be specified in the shared protocol.

## Logging and live transport

### Authoritative microSD log

The microSD log is the full-fidelity record for a run. It must retain all configured signals at their acquisition rates plus events, health records, and session metadata. Live telemetry may be downsampled or filtered later for bandwidth, but SD logging must not inherit that reduction.

Use buffered, append-oriented writes and make interrupted sessions recoverable. The format must be versioned and documented. Tests must cover power-loss/truncated-tail recovery, counter rollover, SD stalls, queue pressure, and a missing or failed card.

### USB serial baseline

USB serial is the first supported live transport. It is read-only telemetry from the vehicle's perspective for the initial release: the dashboard must not need to send control commands to operate the stream. Framing must survive partial reads and allow a receiver to resynchronize after corruption or dropped bytes.

Future LoRa/radio or CAN adapters must implement the same logical record and health model. Transport-specific framing and bandwidth policy belong behind a transport interface; they must not change the meaning of logged signals.

## Shared protocol relationship

The dashboard and firmware must implement one versioned protocol/log specification. The canonical specification should define:

- signal/schema descriptors and stable identifiers;
- scalar, vector, boolean, string/event, and status value types;
- canonical units and display metadata;
- device/session metadata;
- monotonic timestamps, sequence numbers, rollover, and ordering;
- record and packet framing, checksums/integrity checks, and recovery;
- heartbeat and health fields;
- compatibility rules for older logs and unknown future signals;
- golden binary/log fixtures used by both repositories.

The initial canonical specification may live in `lbr-dashboard`, with generated/copied bindings or golden fixtures consumed here. If ownership later becomes awkward, extract it into a small shared repository. Do not allow independent, incompatible packet definitions to evolve in the two codebases.

## Suggested repository layout

```text
CMakeLists.txt
cmake/
docs/
  hardware.md
  protocol.md
include/lbr/
src/
  app/
  acquisition/
  logging/
  protocol/
  transport/
  health/
tests/
  host/
  fixtures/
tools/
.github/workflows/
```

Hardware-specific code should be behind narrow interfaces so protocol, buffering, queue-pressure, and log-recovery behavior can be exercised in host tests without a Pico connected.

## Build workflow

Prerequisites:

1. Install CMake, Ninja (recommended), an ARM GCC toolchain, and the Raspberry Pi Pico SDK.
2. Set `PICO_SDK_PATH` or use the team's documented SDK-fetch mechanism.
3. Configure and build:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

Production/release builds should use a pinned Pico SDK version and a reproducible toolchain in CI. The resulting UF2 is copied to a Pico in BOOTSEL mode. Exact target names will be added with the initial firmware skeleton.

## Test workflow

Before merging firmware changes:

1. run host-side unit tests for protocol encoding/decoding, timestamp/sequence rollover, queues, log recovery, and configuration validation;
2. build the Pico firmware with warnings treated as errors;
3. run formatting and static-analysis checks;
4. replay golden fixtures through the dashboard compatibility tests;
5. complete hardware-in-the-loop smoke tests for BNO055 acquisition, SD logging, USB framing, disconnect/reconnect, and fault reporting when hardware behavior changes.

CI should run all host tests and compile the firmware on every pull request. Hardware-in-the-loop tests may run on a dedicated runner or be recorded as a required release checklist when hardware is unavailable to CI.

## Reliability rules

- Never silently discard samples. Count and report every queue overflow or rejected record.
- Prefer continuing full-fidelity SD logging when the USB host is absent or slow.
- Start a session only after configuration is validated; persist the exact configuration with the log.
- Represent unavailable measurements and calibration explicitly rather than substituting zero.
- Make watchdog resets and brownout/boot reasons observable in the next session.
- Keep protocol compatibility tests synchronized with `lbr-dashboard`.
