# The catalog

`car/catalog.toml` declares the car once: its nodes, signals, CAN data frames, acquisition
slots, the LoRa subset, and the cockpit display list. Everything else is generated from it:

```sh
python3 lbrlib/tools/lbrgen.py car/catalog.toml           # regenerate
python3 lbrlib/tools/lbrgen.py car/catalog.toml --check   # CI: fail if anything is out of date
```

| Generated | What it is |
| --- | --- |
| `car/catalog/include/lbr/signals.def` | `LBR_SIGNAL(NAME, id, TYPE, UNIT, "desc")` rows: the `LBR_SIG_<NAME>` ids |
| `car/catalog/include/lbr/catalog_nodes.def` | node ids, names, and roles: the `LBR_NODE_<NAME>` ids |
| `car/catalog/include/lbr/catalog_frames.def` | the CAN data frame table |
| `car/catalog/include/lbr/catalog_links.def` | the LoRa subset and the cockpit display list |
| `car/catalog/include/lbr/catalog.h` | node id limits |
| `car/nodes/<app>/main/<app>_plan.[ch]` | each firmware's slots (`<APP>_SLOT_<NAME>`) and how each is read |
| `car/nodes/<app>/main/Kconfig` | the menuconfig settings of its `io` slots (`CONFIG_LBR_IO_<SLOT>`) |
| `docs/can-ids.md` (between `lbrgen` markers) | the node and data-frame tables |

Never edit a generated file; edit the TOML and run the generator. The `.def` files are X-macro
data that LBRLib expands, which is how the library stays free of this car's names.

## Nodes

```toml
[node.powertrain]
id = 2
board = "ESP32"
about = "Powertrain Controller: fuel level, engine speed, CVT speed"
slots = ["speeds", "fuel"]
```

| Key | Meaning |
| --- | --- |
| `id` | node id, 1..254. CAN nodes are 1..`can_max`; never renumber or reuse one |
| `board`, `about`, `link` | documentation (the docs tables, generated comments, `lbr_node_about()`) |
| `role` | `"central"` (exactly one node: logger, time master) or `"cockpit"` (at most one; off the bus) |
| `app` | the firmware project under `car/nodes/` (default: the node's name). Nodes sharing an app get one plan array, in id order: the four corners share `corner` |
| `slots` | acquisition slots, in order (see below) |
| `listen_all` | receive every frame (the Central) |
| `health_period_ms` | heartbeat period (default 1000) |

### Slots

A slot is written as its name, and takes its signals from the frames that name it, in frame
order. Write it as a table to list the signals yourself (any order), or for a node off the bus:

```toml
slots = [
  { name = "gps", signals = ["GPS_LATITUDE", "GPS_LONGITUDE", ...] },   # explicit order
  "cockpit_status",                                                     # from its frames
]
```

A slot's rate comes from its frames (they must agree); give `rate_hz` when no frame sets it.

`input` says how the slot is read, and the board binds it by itself:

| `input` | Read by | Set up in |
| --- | --- | --- |
| `"io"` | on-chip ADC, pulse, and switch inputs | menuconfig, `CONFIG_LBR_IO_<SLOT>` (generated, with the channel list and `hint`) |
| `"gps"` | the NMEA GPS receiver (the Central only) | menuconfig, "LBR Central GPS" |
| `"cockpit_link"` | the cockpit's battery and link state (the Central only) | nothing |
| none | a part driver bound in `main.c` with `lbr_bind` | your code ([drivers](drivers.md)) |

```toml
slots = [{ name = "oil", input = "io", hint = "an adc input (0.5..4.5 V pressure sender)" }]
```

## Frames and signals

```toml
[[frame]]
id = 0x200
name = "POWERTRAIN_SPEEDS"
sender = "powertrain"
slot = "speeds"
rate_hz = 100
counter = true
signals = [
  { name = "ENGINE_RPM", id = 0x0200, unit = "rpm", enc = "u16", desc = "Engine speed" },
  { name = "CVT_RPM",    id = 0x0201, unit = "rpm", enc = "u16", desc = "CVT speed" },
]
```

| Key | Meaning |
| --- | --- |
| `id` | 11-bit CAN id in the data range 0x100..0x6FF; lower wins arbitration. Frames are listed in id order |
| `name` | UPPER_CASE frame name |
| `sender` | the node that samples and sends it (never the cockpit) |
| `slot` | the sender's slot whose samples fill it |
| `rate_hz` | how often it is sent; must divide 1000000 |
| `counter` | `true`: a 16-bit sample counter in bytes 6..7 and dlc 8, so receivers rebuild exact sample times and detect lost samples. Fields then have bytes 0..5 |

Each signal:

| Key | Meaning |
| --- | --- |
| `name` | UPPER_CASE; becomes `LBR_SIG_<NAME>` and what `lbr_get(c, NAME)` reads |
| `id` | 16-bit signal id, unique; never renumber or reuse one |
| `type` | `f32` (default), `i32`, `u32`, `bool`, `status` |
| `unit` | `none m_per_s2 rad_per_s deg_c rpm m_per_s percent deg_e7 meter degree count microstrain unix_s bitmask` |
| `enc` | how it travels: `i16 u16 u8 i32 u32` |
| `scale` | physical value = raw x scale (`f32` signals only; default 1) |
| `desc` | one line, ASCII |

Fields are packed from byte 0 in the order listed, little-endian. Each encoding reserves its 8
lowest (signed) or highest (unsigned) raw values for "unavailable, and why", so pick a scale that
leaves headroom: `i16` at 0.01 covers +/-327.6.

Signals that travel in no data frame (the `EVENT` and `FAULT` codes every node sends) are
`[[signal]]` tables with the same keys minus `enc` and `scale`. `EVENT` and `FAULT` must exist.

## Live links

```toml
[lora]
items = [["ENGINE_RPM", 500], ["TEMP_CVT", 1000], ...]   # signal, minimum interval (ms), by priority

[cockpit]
signals = ["ENGINE_RPM", "CVT_RPM", "GPS_SPEED", ...]     # what the cockpit gets, in wire order
```

## What the generator checks

It refuses a catalog, naming the entry, for: duplicate node, signal, or frame ids or names; a
frame outside the data id range or out of order; fields that do not fit the frame (or the bytes
before the counter); a rate that does not divide a second; an integer signal with a scale; a
frame naming a slot its sender lacks, or a slot no frame fills; an explicit slot list that
differs from its frames; nodes sharing an app with different slots; unknown units, types, or
encodings; no `central`, or two; a link list naming an unknown signal; non-ASCII text.

`ctest` then checks what needs the whole system (`test_plans`): every frame has a producer,
each node's plan matches the frames it sends, and the bus stays under its 50 % load ceiling.

## A catalog change is a protocol change

Signal and frame ids, names, types, units, layouts, and rates are folded into
`lbr_catalog_hash()`, which travels in every heartbeat and log, so a log is never decoded with
the wrong catalog. After a change:

1. `python3 lbrlib/tools/lbrgen.py car/catalog.toml`
2. `cmake --build build-dev --target fixtures` (regenerates `dev/tests/fixtures/`, including
   `catalog.json`), then `ctest`
3. update the dashboard app with the new `catalog.json` in the same release
4. flash every node: they must all run the same catalog
