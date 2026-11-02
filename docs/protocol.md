# Protocol

Protocol version **4** (`LBR_PROTOCOL_VERSION`), log format version **1**
(`LBR_LOG_FORMAT_VERSION`).

The dashboard and firmware implement **one versioned protocol/log specification**. This file is
the firmware-side statement of it; the golden fixtures in `dev/tests/fixtures/` pin every byte. Where
this file and the fixtures disagree, the fixtures win and this file has a bug. The CAN-level
details (id map, frame table, field encodings, time grid, bus budget) are in
[`can-ids.md`](can-ids.md); the node topology is [`DAQ-WIRING.md`](../DAQ-WIRING.md).

All multi-byte fields are **little-endian**. All text is ASCII.

## Principles

- **Schema-driven.** Signal identity, value type, and unit come from the catalog
  (`car/components/protocol/include/lbr/signals.def`); which CAN frame, byte, encoding, and scale
  carry a signal come from the frame table (`car/components/protocol/frames.c`). Both are
  exported as `dev/tests/fixtures/catalog.json`, so a decoder needs no hand-written layout.
- **Stable identifiers.** Signal ids and node ids are append-only. Never renumber or reuse one.
- **Explicit unavailability.** A missing measurement is a sample with `valid = false` and a
  reason code in the value slot (on CAN, a reserved raw value per encoding), never a zero.
- **Shared time base.** Every timestamp is microseconds on the Central's monotonic clock (epoch:
  Central boot), distributed over CAN to every node. Sampling is locked to a grid of that clock,
  so a sample's time follows from its grid index. Never packet-arrival time.
- **Count every loss.** Queues are bounded and never block; every dropped sample, frame, record,
  or packet is counted and reported (heartbeats, `LOSS` records, sequence gaps).

## Catalog

`catalog.json` carries, for every signal: `id`, `name`, value `type`, `unit`, and a description;
the frame table (`frames`: id, name, sender, rate, dlc, counter offset, and per field the
signal, byte offset, encoding, scale); the system ids, heartbeat flags, and command codes; the node
table; unit, type, and encoding name lists; event codes; and unavailable-reason names. Signal ids
are grouped like the CAN map, in the high byte:

| Range | Subsystem | Node |
| --- | --- | --- |
| `0x01xx` | Driver inputs | `driver_input` |
| `0x02xx` | Powertrain: engine and CVT speed, fuel | `powertrain` |
| `0x03xx` | IMUs: four suspension (wheel hubs), chassis | `corner_fl` .. `corner_br`, `imu` |
| `0x04xx` | Stress sensors | `stress` |
| `0x05xx` | Temperatures | `temperature` |
| `0x06xx` | GPS, cockpit link | `central` |
| `0x07xx` | Events / faults | any CAN node |

**Catalog hash** (`lbr_catalog_hash`): CRC-32 over, for each signal in id order,
`id u16, type index u8, unit index u8, name, NUL`, then for each frame in id order
`id u16, sender u8, rate u16, dlc u8, counter offset u8 (0xFF none), field count u8`,
each field `signal u16, offset u8, encoding index u8, scale f32 bits`, and `name, NUL`. It travels
in heartbeats (page 2) and every session record, so a node or decoder built against a different
catalog or frame table is detected.

Values are **sensor-frame**: per-node mounting transforms (each corner IMU, the chassis IMU) are
session metadata applied by the dashboard. Signals whose physical location varies (`STRESS_n`)
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
| 3 | 1 | bits 0..3 value type; bit 4 valid; bit 5 synced; bit 6 seq_inferred; bit 7 reserved (0) |
| 4 | 4 | seq |
| 8 | 8 | `t_mono_us`, shared time base |
| 16 | 4 | value bits, or the unavailable reason |

`synced = 0` means the producing node had not yet been disciplined to the Central: its timestamp
is node-local and must not be aligned with other data. `source` is the node that took the sample
(for CAN data, the frame's sender). For a sample decoded from CAN, `seq` is the low 32 bits of
its grid index and `t_mono_us` the grid time; `seq_inferred = 1` means the frame carries no
counter, so seq and time were inferred from the receive time.

## Sequence numbers and rollover

A stream is (source node, signal, medium). Every counter wraps modulo its width and never
resets or saturates within a session; receivers compare with modular arithmetic
(`lbr_seq_delta`): 1 = next, 0 = duplicate, >1 = gap, <0 = reordered.

| Where | Width | Counts |
| --- | ---: | --- |
| `SAMPLE` seq | 32 | grid index of the sample (one per acquisition) |
| CAN data frame counter | 16 | grid index mod 65536 |
| CAN event frame | 6 | events and faults of that node |
| Transport packet / WiFi datagram | 32 | every packet a link accepted (per link and sender) |
| Health counters | 32 | events since boot |

## Heartbeat (health) record

Twelve 8-byte pages; layout in `car/components/health/include/lbr/health.h`. On CAN each page is one
frame at 0x700 + 16 n + p, next to the 2-byte `HEARTBEAT` summary at 0x700 + n (state and error
flags, see `can-ids.md`); in logs and packets the record is `node_id u8` followed by all twelve
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

The Central's authoritative log holds `CAN_FRAME` records: every frame on the bus, byte-exact,
with its receive time, including the Central's own transmissions (flag bit 1: time sync, events,
GPS, cockpit status). Decoding is deterministic from those bytes plus the frame table: a reader
replays the frames in order through the sync-tracking rules of `can-ids.md`, so the log never
depends on the firmware's decoder. Local node logs (a plan with `local_log`; none in the 2026
car) hold `SAMPLE` records at full rate.

**Recovery.** A reader that meets a bad CRC or garbage skips forward to the next valid record and
counts the skipped region. A record that is incomplete at the end of the file (power loss mid-
write) is reported as a truncated tail. Power loss costs at most the record being written plus
whatever had not been fsynced (`CONFIG_LBR_LOG_SYNC_INTERVAL_MS`, default 1 s).

**Gaps.** When the bounded log queue overflows, the dropped records are counted and a `LOSS`
record with the exact count is written ahead of the next record that fits.

Session files are `/sdcard/lbr/NNNNNNNN.LBR`, numbered upward per boot; the firmware never
formats a card or overwrites a session.

## CSV export (lbr-csv version 1)

The SD card holds `.LBR` logs only: they are CRC-checked, survive a power cut mid-write, and
carry the session's config and versions. CSV is the export format for debugging and for the
dashboard app: `python3 dev/tools/lbr_decode.py csv 00000012.LBR` writes `00000012.csv`, with
every record of the log as rows (a long table, not one column per signal, so signals at
different rates need no resampling and nothing is filled in). Each signal in it is a channel
the dashboard app can plot.

Columns, in this order, with a header row:

| column | meaning |
| --- | --- |
| `t_s` | shared time base, seconds with exactly 6 decimals (integer microseconds, no float rounding) |
| `kind` | what the row is (below) |
| `node` | node name from the catalog (`nodeN` if unknown); the logger for session/loss/text rows |
| `name` | signal name, event name, or field name |
| `value` | number, or text for `text`/`command`/`can_raw`; **empty when unavailable**, never 0 |
| `unit` | catalog unit (`m/s^2`, `deg*1e-7`, ...); empty for unitless values |
| `status` | samples: `ok` or the unavailable reason (`NOT_FITTED`, `SENSOR_FAULT`, ...); heartbeats: error flag names joined with `\|` |
| `synced` | samples and events: `1` if `t_s` is on the shared time base, `0` if on the node's own clock |
| `seq` | samples and events: the stream's sequence number (CAN data frames: the grid index) |

| `kind` | rows |
| --- | --- |
| `format` | first row: `lbr_csv_version` = 1 |
| `session` | the SESSION record: `format_version`, `protocol_version`, `fw_version`, `catalog_hash`, `config_crc`, `reset_reason`, `boot_count`, `start_time_us` |
| `config` | one row per `key=value` line of the session config text |
| `sample` | a measurement, from a SAMPLE record or decoded from a CAN data frame |
| `event`, `fault` | `name` = event name, `value` = its argument |
| `heartbeat` | `state` and `err_flags` per heartbeat frame |
| `health` | one row per heartbeat field, from HEALTH records and CAN health pages |
| `command` | Central command frames: `name` = command, `value` = argument bytes (hex) |
| `update` | firmware update (parked, [`can-ids.md`](can-ids.md)): from the Central `name` = op, `value` = target node; from a node `name` = op answered, `value` = result, `status` = its state. Data frames are counted, not listed |
| `can_raw` | any other frame (unknown id, wrong length, extended id): `name` = id, `value` = data (hex) |
| `loss` | LOSS records: `dropped_records`, `dropped_bytes` |
| `text`, `end` | TEXT records; the END record's `records_written` |
| `unknown_record` | a record type this decoder does not know: `name` = type, `value` = payload (hex) |
| `integrity` | last rows: `records`, `corrupt_regions`, `skipped_bytes`, `truncated_tail`, `time_sync_frames`, `time_follow_up_frames`, `update_data_frames` (counted, not listed), `catalog_checked` |

Rules:

- Rows are in log order. Per channel (`node`, `name`) time increases, but rows of different
  channels can be slightly out of time order; sort per channel, not globally.
- Health, text, and unknown records carry no time of their own: they get the time of the
  latest record before them.
- Rows with `synced` = 0 are on a node's free-running clock; do not plot them against the
  shared axis.
- The export refuses a log whose catalog hash differs from the catalog it decodes with (pass
  the right one with `--catalog`), so names are never put on the wrong signals.
- `dev/tests/fixtures/session.csv` is the export of `session.lbr` and pins this format; the
  dashboard app's CSV import must read it.

## Transport packets

Every live link (USB, WiFi, LoRa) and every cockpit datagram carries the same packet:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | version (1) |
| 1 | 1 | link: 0 LoRa, 1 USB, 2 WiFi, 3 BLE, 4 cockpit |
| 2 | 1 | source node (the Central on its links; the sender on the cockpit link) |
| 3 | 1 | record count |
| 4 | 4 | packet seq, per link |
| 8 | 8 | `t_us` when the packet was built |
| 16 | .. | records: `type u8, len u8, payload[len]` |
| end | 4 | CRC-32/IEEE over everything before it |

Record types are the log's, plus `HEALTH_LITE` (0x08: `node u8, state u8, flags u16,
sensor_status u32`), and the dump records `FILE_INFO` (0x10), `FILE_CHUNK` (0x11: `offset u32,
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

A gap in the per-link `seq` at the receiver means packets lost in transit; packets the Central
could not enqueue are counted in its heartbeat (`transport_error_count`). Over WiFi (UDP) packets
can also arrive reordered, so the cockpit link's receivers keep a 32-packet window
(`lbr_seq_window_t`): a packet counts as lost only when it leaves the window unseen, a late one
fills its hole, a repeat is dropped, and a packet older than the window means the sender
restarted its count. Only packets newer than any so far update displayed values, so an older one
never overwrites a newer one.

## Cockpit link

The cockpit display is the only node off the CAN bus. Over 5 GHz WiFi UDP (DAQ-WIRING.md
section 8; `car/components/wlink/include/lbr/wlink.h`), one transport packet per datagram, no COBS:

- **Central -> cockpit** (link 4, port 47801, broadcast on the Central's access point, every
  `cockpit_period_ms`): a `SAMPLE` record per display signal (latest value; stale ones as `NO_DATA`)
  and a `HEALTH_LITE` per CAN node.
- **Cockpit -> Central** (link 4, port 47802, once a second): `SAMPLE` records (`COCKPIT_BATTERY`).
  The Central puts its battery and the link state on CAN (`COCKPIT_STATUS`, 0x60A).

The cockpit keeps no shared clock: it only shows ages. After a receive timeout, or a packet
older than its window (the sender restarted its count), a receiver restarts its window and counts
the packets the old one still held unseen as lost.

USB host commands, one per line: `STAT`, `LIST`, `DUMP <name> [offset]`.

## Time base

The Central is the time master. Ten times per second it sends `TIME_SYNC` (seq, epoch) and then
`FOLLOW_UP` carrying the exact Central time at which `TIME_SYNC` finished transmitting (captured
in the TX-done ISR). Each node timestamps `TIME_SYNC` in its RX ISR and feeds the pair to a PI servo
that tracks offset and rate:

- corrections under 2 ms are slewed; larger ones are stepped and counted (`time_step_count`);
- shared time never runs backwards within an epoch (a backwards correction holds the clock);
- a Central reboot changes the 16-bit epoch, and nodes re-anchor to the new timeline;
- acquisition on every node treats a step or new epoch as a new timeline, not as missed samples.

## Compatibility rules

- Decoders must ignore record types they do not know (skip by length) and must keep samples of
  unknown signals (type `raw`) rather than dropping them.
- Adding a signal, a frame, a node id, an event code, or a record type is backward compatible:
  the catalog hash changes and old ids keep their meaning. Logs of an earlier protocol version
  need that version's decoder (the version is in every session record). Changing any layout in
  this file is incompatible: bump
  `LBR_PROTOCOL_VERSION` (and `LBR_LOG_FORMAT_VERSION` for the log), regenerate the fixtures, and
  update the dashboard app in step.
- Every node and the cockpit report their protocol version and catalog hash; a mismatch is a
  configuration error to surface, not to guess around.

## Golden fixtures

`dev/tests/fixtures/` holds binary fixtures produced by the firmware codecs, each with the expected
decode as JSON: `catalog.json`, `samples.bin`, `health.bin`, `can_capture.bin`, `session.lbr`,
`stream_usb.bin`. CTest fails if a fixture no longer matches the codecs; `dev/tools/lbr_decode.py
--selftest` is an independent decoder that must reproduce every expected JSON and `session.csv`
(the CSV export of `session.lbr`; the `fixtures` target regenerates it too, and ctest runs the
self test as `decoder`). The dashboard app should run the same fixtures through its decoder in
its own CI.

## Still to agree with the dashboard app (#14)

- [ ] where the canonical copy of this spec and the catalog lives (this repo generates it today)
- [ ] the dashboard's fixture replay test, wired into its CI
- [ ] the dashboard app importing `session.csv` (CSV export above) and `.LBR` directly
- [ ] session/topology metadata beyond `key=value` config text (e.g. mounting transforms)
