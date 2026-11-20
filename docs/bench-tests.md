# Bench and car tests

Everything below is implemented, host-tested, and run in the simulator (`dev/sim`); what is left
is proving it on real boards. Work through the sections in order: each one builds on the ones
before it. Record results (numbers, photos of scope traces, the log files) in the tracking issue.

Before starting: every board has had its first-time USB setup (`docs/deploying.md` section 4), the
pins in its sdkconfig match its schematic (`docs/hardware.md`), and you have a laptop with ESP-IDF
v6.1 and `pip install pyserial`. `PORT` below is the Central's USB serial port.

## 1. CAN bus

Setup: the Central and at least two sensor nodes on one bus, 120 ohm at both physical ends.

1. Power everything. `python3 dev/tools/lbr_decode.py serial PORT` shows frames from every node.
2. `python3 dev/tools/lbr_deploy.py status --port PORT` lists every node with its firmware.
3. Unplug one node's CAN cable for 10 s, then plug it back: the Central logs `NODE_LOST` about
   3 to 4 s after the unplug and `NODE_FOUND` when it is back.
4. Short CAN_H to CAN_L for 2 s on a running bus: every node reports `CAN_BUS_OFF`, recovers on
   its own, and keeps publishing. Repeat 5 times; `STAT` must show no stuck transmit queue
   (`can tx_dropped` stops growing once the bus is back). On a classic ESP32 node this is also
   the test of the bus-off transmit-slot recovery, which needs real hardware.
5. With every node on the bus, `lbr_decode.py serial PORT --cmd STAT`: the Central's line ends
   with `bus NN.N%`, the load it measured over the last second, counted like the plan (worst-case
   bit stuffing, about 46 % for the full car; `ctest -R plans -V` prints the plan per node). It
   must stay under 50 %. `STAT` also lists any node expected but never heard as `MISSING`.

Pass: every node's frames decode at the Central, lost/found and bus-off recover with no reboot,
load under 50 %.

## 2. Time sync

Setup: two boards on the bus (the Central and a node, then two nodes), each built with
`CONFIG_LBR_SYNC_PROBE_GPIO` set to a free pin (`lbr_deploy.py menuconfig TARGET`, "LBR CAN bus").
Each board then pulses that pin for 100 us at every whole second of the shared clock.

1. Put both probe pins on a two-channel scope, trigger on one, and measure the distance between
   the rising edges over a few minutes (persistence mode).
2. Repeat with a node 30 cm from a heat gun (crystal drift) and after power-cycling the Central
   (new epoch: the node must re-sync within a few seconds and its probe resumes).

Pass: edge distance under the agreed bound (target 100 us) on every pair, CAN nodes and the
corner hubs. Turn the probe off again (-1) for the car.

## 3. The Central's log (SD card)

1. Run the full node set for 10 minutes with USB, WiFi, LoRa, and BLE all active.
2. `python3 dev/tools/lbr_decode.py serial PORT --cmd LIST`, then
   `--cmd "DUMP <name>" --out run.lbrlog`: the tool checks the CRC and says "verified".
3. `python3 dev/tools/lbr_decode.py log run.lbrlog`: no corrupt regions, `LOSS` records only if
   a heartbeat also reported a loss. `python3 dev/tools/lbr_decode.py csv run.lbrlog` exports it.
4. `lbr_replay run.lbrlog --check` (host build, `dev/tools/lbr_replay`): bit for bit.
5. Pull the card mid-session: the Central reports `SD_WRITE_ERROR` and DEGRADED; put it back:
   `SD_MOUNTED` within about 5 s and a new session file.
6. Cut the Central's power mid-session. On the next boot the old file reads back up to a few
   records before the cut (`truncated tail`, never a corrupt file), and the new boot logs its
   reset reason.

Pass: every step as described; the authoritative log never loses data a link was fine with.

## 4. USB link

1. Live view: `lbr_decode.py serial PORT` while the car runs; no "packets lost on the link".
2. Dump a long session (step 3.2) and compare it with the card read on a laptop: identical.

## 5. WiFi pit link

Setup: the Central with a WiFi password set locally; a laptop joined to its access point
(5 GHz by default).

1. `curl http://<central>/status` and `curl http://<central>/logs`; download a session with
   `curl -o run.lbrlog http://<central>/logs/<name>` and compare it with the USB dump.
2. Live stream: `nc <central> <stream port> > stream.bin`, then
   `lbr_decode.py stream stream.bin`.
3. `curl http://<central>/config`, change one value, `POST` it back: rejected if invalid, applied
   at the next session if valid.
4. OTA: `curl -H "X-LBR-Confirm: reboot" --data-binary @build/lbr_central.bin http://<central>/ota`.
   Then push a deliberately broken image (one that panics at boot): the bootloader rolls back
   and the Central boots the previous image.
5. The cockpit and the laptop joined at the same time; the cockpit display keeps updating.

## 6. BLE

With WiFi and CAN active, read the status and version characteristics from a phone (for example
nRF Connect). Pass: node states, sync, SD, bus health, and calibration are readable.

## 7. LoRa

Setup: the Central's radio and a pit receiver, region settings confirmed.

1. On the bench: packets decode and their sequence numbers have no gaps.
2. On the course: drive the full loop; record packet loss against distance. Tune SF, bandwidth,
   and duty in menuconfig if needed ("LBR LoRa").

Pass: the pit decodes the in-race subset during a run and counts every lost packet.

## 8. Each node

For every node, with its parts fitted:

1. Every channel at its planned rate with shared-timeline timestamps (`lbr_decode.py log`: no
   `[unsynced time]` after the first second, no gaps in the sample counters).
2. Heartbeat RUN, or DEGRADED with the reason if a slot is not fitted.
3. Unplug a sensor: its channels read `SENSOR_FAULT` (never 0) and the node reports DEGRADED.
4. Update it over CAN (`lbr_deploy.py can --port PORT TARGET`): it confirms itself within 60 s.

## 9. Cockpit

The cockpit joins the Central's access point, shows live values, marks stale values as stale
when the Central is switched off, and reports its battery (COCKPIT_BATTERY in the Central's log).
