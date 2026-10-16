# Hardware

> **Status: not finalized.** Per-node GPIO assignments are TBD and must come from
> each node's schematic. Do not hard-code pins or assign line semantics until the
> electronics team confirms them.

## Nodes (starting grouping)

Grouped by vehicle zone to minimize harness length. Rename/split as the design settles.

| Node | Zone | Sensors (parts TBD) |
| --- | --- | --- |
| `gateway` | cockpit / dash | none — CAN sniffer, microSD logger, telemetry (LoRa/USB/WiFi/BLE), time-sync source |
| `powertrain` | engine bay | engine RPM, engine/CVT temperatures, fuel level |
| `dynamics` | chassis / CoG | IMU (accel/gyro/orientation), vehicle/wheel speed |

## Per-node interfaces (minimum)

| Function | Interface | Notes |
| --- | --- | --- |
| CAN TX / RX | TWAI | External CAN transceiver (e.g. SN65HVD230 / TJA1051); GPIO **TBD** |
| Sensor bus(es) | I2C / SPI / analog | Depends on the node's sensors; **TBD** |
| microSD (gateway, local-logging nodes) | SPI / SDMMC | **TBD** |
| LoRa radio (gateway) | SPI (+ control) | SX127x / RFM95-class module; **TBD** |

## To verify against the final schematic/harness before bring-up

Voltage levels, transceiver choice, pull-ups, CAN bus bit rate, termination (120 Ω at
both physical ends) and topology, card-detect, interrupt, reset, and power-control lines.
