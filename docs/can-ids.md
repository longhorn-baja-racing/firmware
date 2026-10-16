# CAN ID / arbitration map

> **Status: draft.** This map is part of the shared protocol version. Keep it synchronized
> with `lbr-dashboard` and with every node's firmware.

CAN is the backbone: nodes are decoupled publishers/subscribers, not point-to-point links.
Lower CAN IDs win arbitration, so **safety/alarm and time-sync frames get the lowest IDs**.

## Priority bands (proposed)

| Band | ID range | Purpose |
| --- | ---: | --- |
| Critical | `0x000`-`0x00F` | Faults, alarms, emergency/safety events |
| Time sync | `0x010`-`0x01F` | Gateway time-sync broadcast |
| Powertrain | `0x100`-`0x1FF` | RPM, temps, fuel level |
| Dynamics | `0x200`-`0x2FF` | IMU, speed |
| Electrical | `0x300`-`0x3FF` | Battery, voltages |
| Health | `0x700`-`0x7FF` | Per-node heartbeat/status |

## To define

- [ ] exact frame ID per signal / record
- [ ] source node identity encoding
- [ ] classic CAN vs CAN FD payload layout
- [ ] bus bit rate and segmentation strategy for multi-frame records
