# Logging and data

## The SD log

The Central writes one log per boot: `/sdcard/lbr/NNNNNNNN.lbrlog`. It holds:

- every CAN frame on the bus, byte for byte, with its receive time on the shared clock,
  including the Central's own (time sync, events, GPS, cockpit status);
- every car logic output, as `KEY` and `VALUE` records;
- firmware log lines (`TEXT`), loss markers (`LOSS`, with exact counts), and a clean-close
  `END`;
- first, a `SESSION` record: firmware and protocol versions, the catalog hash, the reset reason,
  boot count, and the exact validated configuration.

Each record carries a CRC. A reader skips damage and counts it; a log cut by a power loss is
read up to its last complete record (at most about a second of data, the fsync interval, is
lost). The log goes through a bounded queue: a slow card or a busy link never blocks
acquisition, and anything the queue cannot take is counted and marked in the log. Format:
[`docs/protocol.md`](../../docs/protocol.md).

## Getting logs off the car

```sh
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd LIST                         # list sessions
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd "DUMP 00000012.lbrlog" --out run.lbrlog
```

or pull the card.

## Reading them

```sh
python3 dev/tools/lbr_decode.py log run.lbrlog          # human-readable, every record
python3 dev/tools/lbr_decode.py log run.lbrlog --json   # the same, as JSON
python3 dev/tools/lbr_decode.py csv run.lbrlog          # -> run.csv for the dashboard app
```

The CSV is a long table, one row per value: `t_s,kind,node,name,value,unit,status,synced,seq`.
Samples are `kind=sample`, car logic outputs `kind=output` (`name` = the key), and events,
faults, heartbeats, and health fields have their own kinds. An unavailable value has an empty
`value` and its reason in `status`: never 0. The export counts anything it leaves out in its
closing `integrity` rows. The decoder refuses a log written with a different catalog, so names
never land on the wrong signals.

## Live data

The Central streams the same records live:

| Link | What | Tool |
| --- | --- | --- |
| USB | every frame and output, batched | `lbr_decode.py serial /dev/ttyUSB0` |
| WiFi (pit) | the same stream, over the Central's access point | the dashboard app |
| LoRa | a prioritized, rate-limited subset (`[lora]` in the catalog), health, and faults | the pit receiver |
| Cockpit | the display list (`[cockpit]` in the catalog) and node status, 10 Hz | the cockpit node |
| BLE | quick status at the car | a phone |

Every packet carries a sequence number, so a receiver detects loss, duplicates, and reordering.

## Replay

Like AdvantageKit's, LBRLib's replay re-runs the car logic on a recorded run: try a fix to a task,
or add an output, and see what it would have said on that run.

```sh
cmake --build build-dev --target lbr_replay
build-dev/tools/lbr_replay/lbr_replay run.lbrlog replayed.lbrlog
python3 dev/tools/lbr_decode.py csv replayed.lbrlog     # original and Replay/... side by side
```

`replayed.lbrlog` is the run with the replayed outputs added as `Replay/<key>` (for example
`Replay/CVT/Ratio` next to `CVT/Ratio`), so the dashboard plots both. The tool also compares them
value by value and prints the first difference:

```text
run.lbrlog: 26317 records, 23203 frames, 1500 logic ticks, 15 heartbeats
outputs: 1500 recorded, 1500 replayed
replay matches the recording, bit for bit
```

With the logic unchanged, the replay is identical to the recording, bit for bit: `ctest` checks
it on every simulator scenario (`replay_<name>`). `--check` makes any difference an error, and
`--drop-task NAME` replays without one task, to see what it contributes.

### Why it is exact

Car logic only reads through `lbr_get` and only writes through `lbr_out`, on one tick, so
everything it depends on is in the log, in the order it saw it:

- **every CAN frame**, byte for byte with its receive time. The Central logs a frame and feeds
  it to the logic under one lock, so the log's order is exactly the logic's order;
- **when each logic tick ran**, and **when the heartbeat marked nodes lost or missing** (`TICK`
  records, about 1.2 KB/s);
- across boards, **CAN arbitration** already puts every frame in one order; sensor nodes are I/O
  devices, so their frames are the inputs and their firmware needs no replaying.

The replay feeds those records through the same decoder, node table, and logic runner the
Central uses (`lbr/replay.h`; the Central calls the same functions), with the car logic of the
build you run it from.

### Limits

- **A log opened mid-session** (after the SD card came back) starts without the logic's history,
  so the replay starts cold and can differ until the logic warms up; the tool says so. The
  session's first file always replays from the start.
- **A log cut off by a power loss** can end between a logic tick and its outputs; the replay
  runs that last tick in full and does not count the outputs the cut took as differences.
- **A replayed file replays again**: its earlier `Replay/...` records are left out (and counted).
- **Anything the log lost** (a `LOSS` record: the queue overflowed, or the card was out) is lost
  to the replay too.
- **The catalog must match**: replay with the catalog of the firmware that wrote the log.
- Replay covers the **car logic**. The rest of the Central (the links, the cockpit, time sync)
  is not replayed; its effects are already in the log as frames.
