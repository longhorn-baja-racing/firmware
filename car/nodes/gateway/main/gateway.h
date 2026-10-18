#pragma once

/*
 * Gateway / logger node. Responsibilities (README "Gateway / logger node"):
 *   - sniff the entire CAN bus and write the authoritative, time-aligned,
 *     full-fidelity microSD log (every frame, raw, with its receive time);
 *   - be the time master: broadcast TIME_SYNC + FOLLOW_UP;
 *   - drive the live links: LoRa in-race subset; USB / WiFi full stream;
 *     BLE status;
 *   - track node presence and health and report losses.
 * Every path between them is a bounded, non-blocking queue: a slow or absent
 * live link never reduces what reaches the SD card.
 */
void gateway_run(void);
