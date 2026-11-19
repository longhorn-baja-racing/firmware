# Logging and data

## The SD log

The Central writes one log per boot: `/sdcard/lbr/NNNNNNNN.LBR`. It holds:

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
python3 dev/tools/lbr_decode.py serial /dev/ttyUSB0 --cmd "DUMP 00000012.LBR" --out run.lbr
```

or pull the card.

## Reading them

```sh
python3 dev/tools/lbr_decode.py log run.lbr          # human-readable, every record
python3 dev/tools/lbr_decode.py log run.lbr --json   # the same, as JSON
python3 dev/tools/lbr_decode.py csv run.lbr          # -> run.csv for the dashboard app
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

AdvantageKit's replay re-runs robot code on a recorded log to try a fix or add an output after
the fact. LBRLib is built toward the same, with a `.lbrlog` file:

| Step | Status |
| --- | --- |
| Every CAN frame logged byte-exact, in bus order, on the shared clock | done |
| Car logic reads only through `lbr_get` and writes only through `lbr_out`, on one tick | done: tasks you write today are replayable as they are |
| Outputs logged with their status (`KEY` / `VALUE`) | done |
| The Central's whole core on one deterministic cycle, logging its non-CAN inputs (cockpit packets, GPS bytes, clock reads) | planned |
| `.lbrlog` (log format 2): the cycle records, self-describing keys | planned |
| `lbr_replay in.lbrlog out.lbrlog`: re-run the logic, write `/Replay/...` outputs next to the originals | planned |
| Replay in the simulator and the dashboard app, side by side with the original | planned |

The ordering design is in [concepts](concepts.md#ordering-and-replay).
