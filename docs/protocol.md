# Protocol

Protocol version **2** (`LBR_PROTOCOL_VERSION`), log format version **1**
(`LBR_LOG_FORMAT_VERSION`).

The dashboard and firmware implement **one versioned protocol/log specification**. This file is
the firmware-side statement of it; the golden fixtures in `dev/tests/fixtures/` pin every byte. Where
this file and the fixtures disagree, the fixtures win and this file has a bug. The CAN-level
details (ID layout, frame payloads, bus budget) are in [`can-ids.md`](can-ids.md).

All multi-byte fields are **little-endian**. All text is ASCII.

## Principles

- **Schema-driven.** Signal identity, value type, unit, and CAN priority come from the catalog
  (`car/components/protocol/include/lbr/signals.def`, exported as `dev/tests/fixtures/catalog.json`).
  Nothing about a signal is implied by its position in a frame.
- **Stable identifiers.** Signal ids and node ids are append-only. Never renumber or reuse one.
- **Explicit unavailability.** A missing measurement is a sample with `valid = false` and a
  reason code in the value slot, never a zero.
- **Shared time base.** Every timestamp is microseconds on the gateway's monotonic clock (epoch:
  gateway boot), distributed over CAN. Never packet-arrival time.
- **Count every loss.** Queues are bounded and never block; every dropped sample, frame, record,
  or packet is counted and reported (heartbeats, `LOSS` records, sequence gaps).

## Catalog

`catalog.json` carries, for every signal: `id`, `name`, value `type`, `unit`, `can_prio`, and a
description, plus the node table, unit and type name lists, event codes, and unavailable-reason
names. Ids are grouped by subsystem in the high byte:

| Range | Subsystem | Owner node |
| --- | --- | --- |
| `0x01xx` | IMU / inertial | `imu_gps` |
| `0x02xx` | Powertrain / speed, fuel | `fuel_gauge` (fuel); RPM and wheel speed unassigned |
| `0x03xx` | Temperatures | `temperature` |
| `0x04xx` | Electrical | unassigned |
| `0x05xx` | GPS | `imu_gps` |
| `0x06xx` | Suspension | `suspension` |
| `0x07xx` | Events / faults | any node |
| `0x08xx` | Driver inputs | `driver_inputs` |
| `0x09xx` | Structural strain | `stress` |
| `0x0Axx` | Cockpit dashboard | `dashboard` |

**Catalog hash** (`lbr_catalog_hash`): CRC-32 over, for each signal in id order,
`id u16, type index u8, unit index u8, can_prio u8, name, NUL`. It travels in heartbeats (page 2)
and every session record, so a node or decoder built against a different catalog is detected.

Values are **sensor-frame**: per-node mounting transforms are session metadata applied by the
dashboard. Signals whose physical location varies (`TEMP_AUX_n`, `STRAIN_n`, `DASH_BUTTONS` bits)
get their meaning from the session config.

## Value types and unavailability

| Type | Index | Value bits |
| --- | --- | --- |
| `none` | 0 | reserved |
| `f32` | 1 | IEEE-754 single |
| `i32` | 2 | two's complement |
| `u32` | 3 | unsigned |
| `bool` | 4 | 0 or 1 |
| `status` | 5 | enumerated code |
| `raw` | 6 | decoder did not know the signal; 32 bits preserved |

When `valid = false` the value bits hold the reason: 0 `UNSPECIFIED`, 1 `NOT_FITTED`,
2 `NO_DATA`, 3 `SENSOR_FAULT`, 4 `OUT_OF_RANGE`, 5 `NOT_CALIBRATED`, 6 `BUS_ERROR`.

`EVENT` and `FAULT` samples are `status` values: `code` in bits 0..15, argument in bits 16..31.
Codes are listed in `catalog.json` (`events`).

## Sample record (20 bytes)

Used in logs (`SAMPLE` records, local node logs) and transport packets.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | signal id |
| 2 | 1 | source node id |
| 3 | 1 | bits 0..3 value type; bit 4 valid; bit 5 synced; bit 6 seq_short; bit 7 reserved (0) |
| 4 | 4 | seq |
| 8 | 8 | `t_mono_us`, shared time base |
| 16 | 4 | value bits, or the unavailable reason |

`synced = 0` means the producing node had not yet been disciplined to the gateway: its timestamp
is node-local and must not be aligned with other data. `seq_short = 1` means only the low 6 bits
of `seq` are meaningful (sample decoded from CAN).

## Sequence numbers and rollover

A stream is (source node, signal, medium). Every counter wraps modulo its width and never
resets or saturates within a session; receivers compare with modular arithmetic
(`lbr_seq_delta`): 1 = next, 0 = duplicate, >1 = gap, <0 = reordered.

| Where | Width | Counts |
| --- | ---: | --- |
| Local log `SAMPLE` | 32 | every acquisition |
| CAN sample frame | 6 | every published frame of that stream |
| Transport packet | 32 | every packet a link accepted (per link) |
| Health counters | 32 | events since boot |

## Heartbeat (health) record

Twelve 8-byte pages; layout in `car/components/health/include/lbr/health.h`. On CAN each page is one
frame (see `can-ids.md`); in logs and packets the record is `node_id u8` followed by all twelve
pages (97 bytes). Reserved fields are zero and a decoder rejects a page where they are not.
`*_age_ms = 0xFFFFFFFF` means "not applicable" (e.g. no SD card on that node).

The reset reason of the boot (power-on, software, panic, watchdog, brownout, deep sleep,
external) is in page 0 and in every session record, so a crash is visible in the next session.

## Log format (version 1)

A log is a sequence of self-delimiting records:

```text
record := 0xA5 0x5A  type u8  flags u8 (=0)  len u16  payload[len]  crc32 u32
crc32  := CRC-32/IEEE over type, flags, len, payload
```

| Type | Name | Payload |
| ---: | --- | --- |
| 0x01 | `SESSION` | always first; see below |
| 0x02 | `SAMPLE` | sample record (20 bytes) |
| 0x03 | `HEALTH` | health record (97 bytes) |
| 0x04 | `CAN_FRAME` | raw bus frame (22 bytes, `can-ids.md`) |
| 0x05 | `LOSS` | `dropped_records u32, dropped_bytes u32, t_us u64` |
| 0x06 | `TEXT` | ASCII diagnostic text (firmware log lines) |
| 0x07 | `END` | `t_us u64, records_written u32` (clean close) |

`SESSION` payload: `format_version u16, protocol_version u16, fw_version u32, catalog_hash u32,
config_crc u32, node_id u8, reset_reason u8, reserved u16, boot_count u32, start_time_us u64`,
then the exact validated configuration as `key=value` lines (`config_crc` is its CRC-32).

The gateway's authoritative log holds `CAN_FRAME` records: every frame on the bus, byte-exact,
with its receive time, including the gateway's own transmissions (flag bit 1). Decoding is
deterministic from those bytes, so the log never depends on the firmware's decoder.
Local node logs (redundancy nodes) hold `SAMPLE` records at full rate.

**Recovery.** A reader that meets a bad CRC or garbage skips forward to the next valid record and
counts the skipped region. A record that is incomplete at the end of the file (power loss mid-
write) is reported as a truncated tail. Power loss costs at most the record being written plus
whatever had not been fsynced (`CONFIG_LBR_LOG_SYNC_INTERVAL_MS`, default 1 s).

**Gaps.** When the bounded log queue overflows, the dropped records are counted and a `LOSS`
record with the exact count is written ahead of the next record that fits.

Session files are `/sdcard/lbr/NNNNNNNN.LBR`, numbered upward per boot; the firmware never
formats a card or overwrites a session.

## Transport packets

Every live link (USB, WiFi, LoRa) carries the same packet:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | version (1) |
| 1 | 1 | link: 0 LoRa, 1 USB, 2 WiFi, 3 BLE |
| 2 | 1 | source node (gateway) |
| 3 | 1 | record count |
| 4 | 4 | packet seq, per link |
| 8 | 8 | `t_us` when the packet was built |
| 16 | .. | records: `type u8, len u8, payload[len]` |
| end | 4 | CRC-32/IEEE over everything before it |

Record types are the log's, plus `HEALTH_LITE` (0x08: `node u8, state u8, flags u16,
sensor_status u32`) and the dump records `FILE_INFO` (0x10), `FILE_CHUNK` (0x11: `offset u32,
data`), `FILE_END` (0x12: `bytes u32, crc32 u32, name`).

- **USB and WiFi** are byte streams: each packet is COBS-encoded and followed by `0x00`. A
  receiver splits at `0x00`, so it resynchronizes after any corruption (boot-ROM text and panic
  output on the serial port are simply rejected frames). They carry the raw `CAN_FRAME` records,
  i.e. the same data as the authoritative log, batched every `stream_batch_ms`.
- **LoRa** sends one packet per radio packet (max 255 bytes, no COBS): faults/events first, then
  compact health, then the latest value of each in-race subset signal (see
  `car/components/transport/include/lbr/lora_policy.h`). Downsampling happens here only; the SD log
  keeps everything.
- **BLE** is read-only GATT (status and version text), no stream.

A gap in the per-link `seq` at the receiver means packets lost in transit; packets the gateway
could not enqueue are counted in its heartbeat (`transport_error_count`).

USB host commands, one per line: `STAT`, `LIST`, `DUMP <name> [offset]`.

## Time base

The gateway is the time master. Four times per second it sends `TIME_SYNC` (seq, epoch) and then
`FOLLOW_UP` carrying the exact gateway time at which `TIME_SYNC` finished transmitting (captured in
the TX-done ISR). Each node timestamps `TIME_SYNC` in its RX ISR and feeds the pair to a PI servo
that tracks offset and rate:

- corrections under 2 ms are slewed; larger ones are stepped and counted (`time_step_count`);
- shared time never runs backwards within an epoch (a backwards correction holds the clock);
- a gateway reboot changes the 16-bit epoch, and nodes re-anchor to the new timeline.

## Compatibility rules

- Decoders must ignore record types they do not know (skip by length) and must keep samples of
  unknown signals (type `raw`) rather than dropping them.
- Adding a signal, node id, event code, or record type is backward compatible: append only, and
  the catalog hash changes. Changing any layout in this file is incompatible: bump
  `LBR_PROTOCOL_VERSION` (and `LBR_LOG_FORMAT_VERSION` for the log), regenerate the fixtures, and
  update `lbr-dashboard` in step.
- Every node and the dashboard report their protocol version and catalog hash; a mismatch is a
  configuration error to surface, not to guess around.

## Golden fixtures

`dev/tests/fixtures/` holds binary fixtures produced by the firmware codecs, each with the expected
decode as JSON: `catalog.json`, `samples.bin`, `health.bin`, `can_capture.bin`, `session.lbr`,
`stream_usb.bin`. CTest fails if a fixture no longer matches the codecs; `dev/tools/lbr_decode.py
--selftest` is an independent decoder that must reproduce every expected JSON. `lbr-dashboard`
should run the same fixtures through its decoder in its own CI.

## Still to agree with lbr-dashboard (#14)

- [ ] where the canonical copy of this spec and the catalog lives (this repo generates it today)
- [ ] the dashboard's fixture replay test, wired into its CI
- [ ] session/topology metadata beyond `key=value` config text (e.g. mounting transforms)
