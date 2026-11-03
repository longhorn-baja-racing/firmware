# Deploying firmware to the car

This is the step-by-step guide for putting a firmware release on the car. Follow it in order and
do not skip steps. If anything prints `ERROR`, stop and look it up in
[If something goes wrong](#6-if-something-goes-wrong).

Contents:

1. [How each board is updated](#1-how-each-board-is-updated)
2. [Rules](#2-rules)
3. [One-time laptop setup](#3-one-time-laptop-setup)
4. [First-time setup of a board (USB, once per board)](#4-first-time-setup-of-a-board-usb-once-per-board)
5. [Routine deploy (every new release)](#5-routine-deploy-every-new-release)
6. [If something goes wrong](#6-if-something-goes-wrong)
7. [Updating the cockpit display](#7-updating-the-cockpit-display)
8. [Going back to an earlier release](#8-going-back-to-an-earlier-release)
9. [How it works](#9-how-it-works)

## 1. How each board is updated

| Board | Target name | How it is updated | How often |
| --- | --- | --- | --- |
| Central Controller | `central` | USB cable to the Central | every deploy |
| IMU Controller | `imu` | over CAN, through the Central | every deploy |
| Powertrain Controller | `powertrain` | over CAN | every deploy |
| Driver Input Controller | `driver_input` | over CAN | every deploy |
| Temperature Controller | `temperature` | over CAN | every deploy |
| Stress Controller | `stress` | over CAN | every deploy |
| Wheel hubs FL, FR, BL, BR | `corner_fl`, `corner_fr`, `corner_bl`, `corner_br` | over CAN | every deploy |
| Cockpit display | `cockpit` | USB cable to the cockpit | only when its firmware changes |

You plug the laptop into **one** board, the Central. The Central passes the new firmware to the
nine CAN nodes over the bus. Two exceptions need a cable to the board itself:

- **The very first flash of any board**, including a replacement board (section 4).
- **The cockpit display**, which is not on CAN. It is updated over USB only, on purpose: its WiFi
  link is not used for flashing (section 7).

Every command below is `python dev/tools/lbr_deploy.py ...`, run from the root of the firmware
repository. `python dev/tools/lbr_deploy.py list` prints the table above.

## 2. Rules

1. **Only with the car parked.** Never during a run, and never with the engine running.
2. **Keep power stable.** Use a charger or a full battery. Do not switch the car off until the
   command prints `DEPLOY COMPLETE` (it is safe if it happens by accident, see section 6, but it
   wastes time).
3. **Deploy every board from the same commit.** The nodes and the Central must agree on the
   protocol. The tool updates all CAN nodes together for this reason.
4. **For a race, deploy a committed release.** If the tool prints `WARNING: built from
   uncommitted changes`, the version on the car cannot be traced. That is fine on the bench, not
   for a race.
5. **Use the deploy laptop.** Pin and input settings (menuconfig) are stored on the laptop that
   set them, not in git (see section 3, step 5). A different laptop builds firmware without them.

## 3. One-time laptop setup

1. Install ESP-IDF v6.1 as described in the [README](../README.md#1-prerequisites).
2. Clone the firmware repository:
   `git clone git@github.com:longhorn-baja-racing/firmware.git`
3. Every time you open a new terminal for deploying, activate ESP-IDF first:

   ```sh
   . ~/.espressif/tools/activate_idf_v6.1.sh
   cd firmware
   ```

   This makes `python` the ESP-IDF Python, which already includes `pyserial`. Run every command in
   this guide in that terminal.
4. Learn how to find a board's USB port. Unplug the board, run the command below, plug it in, and
   run it again. The new entry is the port.

   | Laptop | Command | Port looks like |
   | --- | --- | --- |
   | Linux | `ls /dev/ttyUSB* /dev/ttyACM*` | `/dev/ttyUSB0` |
   | macOS | `ls /dev/cu.*` | `/dev/cu.usbserial-0001` |
   | Windows | Device Manager, "Ports (COM & LPT)" | `COM3` |

   On Linux, if you get "permission denied" on the port, run `sudo usermod -aG dialout $USER`
   (or `uucp` on Arch-based systems), then log out and back in.
5. **Pin and input settings.** Until the schematics are final, pins default to "not set"
   ([`hardware.md`](hardware.md)). They are set once per target with
   `python dev/tools/lbr_deploy.py menuconfig <target>`, and saved on this laptop in
   `car/nodes/<node>/sdkconfig` (for the wheel hubs, `car/nodes/corner/sdkconfig.corner_fl` and so
   on). These files are not in git. **Back them up**: if they are lost, a build has no pins and the
   node reports `NOT_FITTED` / `CAN_PINS_UNSET`.

## 4. First-time setup of a board (USB, once per board)

Do this once for every new or replacement board, and again only if a release note says
**"USB flash required"** (the flash layout or bootloader changed). Updates over CAN work only after
this step.

For each board:

1. **Make sure you have the right target.** For the wheel hubs, the target must match where the
   board is mounted: `corner_fl` is front left, `corner_fr` front right, `corner_bl` back left,
   `corner_br` back right. Label each hub board with its target name. A hub flashed with the
   wrong corner takes that corner's identity on the bus.
2. Connect the laptop to **that board's** USB port, and find the port (section 3, step 4).
3. Run (example for the powertrain board on `/dev/ttyUSB0`):

   ```sh
   python dev/tools/lbr_deploy.py usb powertrain --port /dev/ttyUSB0
   ```

   This builds the firmware and flashes it. It must end with `DONE: powertrain now runs ...`.
   If it cannot connect, hold the board's BOOT button, press and release RESET, release BOOT, and
   run the command again.
4. Unplug, and go to the next board.

When every board is flashed, connect the car's CAN harness, power the car, plug the laptop into
the Central, and check:

```sh
python dev/tools/lbr_deploy.py status --port /dev/ttyUSB0
```

Every node must show `confirmed yes`. A node showing `(no answer: not on the bus?)` is not
powered, not connected to CAN, or has CAN pins not set (section 3, step 5).

## 5. Routine deploy (every new release)

### Before you start

- [ ] The car is parked and powered, on a charger or a full battery.
- [ ] Every board has had its first-time setup (section 4).
- [ ] The laptop is charged, and ESP-IDF is active in your terminal (section 3, step 3).

### Steps

1. **Get the release.** Replace `<release>` with the commit or tag the team agreed on:

   ```sh
   git fetch
   git checkout <release>
   git status
   ```

   `git status` must say `nothing to commit, working tree clean`.

2. **Build every board's firmware:**

   ```sh
   python dev/tools/lbr_deploy.py build
   ```

   This takes a few minutes. It must end with `Built: central, imu, powertrain, ...`. If any build
   fails, stop: nothing on the car has changed.

3. **Plug the laptop into the Central's USB port** and find the port (section 3, step 4). The
   examples below use `/dev/ttyUSB0`.

4. **Flash the Central:**

   ```sh
   python dev/tools/lbr_deploy.py usb central --port /dev/ttyUSB0
   ```

   It must end with `DONE: central now runs ...`. The Central reboots (about 5 seconds); the other
   nodes keep running and resync to it on their own.

5. **Update the nine CAN nodes:**

   ```sh
   python dev/tools/lbr_deploy.py can --port /dev/ttyUSB0
   ```

   The tool lists the images, then asks you to confirm the car is parked. Type `yes`. It then runs
   four phases:

   | Phase | What happens | If it fails |
   | --- | --- | --- |
   | `[1/4] Checking every node` | asks each node what it runs; nodes already on this build are skipped | nothing changed |
   | `[2/4] Sending images` | sends each image (about 20 s per node); the nodes keep running their old firmware | nothing changed: every node still runs its old firmware |
   | `[3/4] Switching` | every node that received an image reboots into it, all at once | see section 6 |
   | `[4/4] Waiting for each node` | each node must sync to the Central and confirm itself | see section 6 |

   A good run ends with:

   ```text
   Result:
     imu          OK
     powertrain   OK
     ...
   DEPLOY COMPLETE: every node runs the new firmware.
   ```

   If a node has not been built into the car yet, the tool stops in phase 1 and lists it.
   To update the others, run the same command with `--skip-missing`.

6. **Check the result:**

   ```sh
   python dev/tools/lbr_deploy.py status --port /dev/ttyUSB0
   ```

   Every node must show `confirmed yes` and `same as local build`.

7. **Power-cycle test.** Switch the car off, wait 10 seconds, switch it on, and run step 6 again.
   The result must be the same. This proves the new firmware starts on its own.

8. **Cockpit:** only if this release changes the cockpit firmware (the release notes say so),
   follow section 7.

9. **Write it down.** Record the commit you deployed and the date in the team's car log.

## 6. If something goes wrong

**Is the car safe to drive?** Every node always runs one complete firmware: either the old one or
the new one, never half of each. A node only switches after the whole image arrived and passed its
checks, and a new image that does not work goes back to the old one by itself. What you must avoid
is racing with a **mix** of releases: finish the deploy, or go back to the earlier release on every
board (section 8).

| What you see | What it means | What to do |
| --- | --- | --- |
| `ESP-IDF is not active` | the terminal has no ESP-IDF | run `. ~/.espressif/tools/activate_idf_v6.1.sh` |
| `no build of <target>` | step 2 was skipped or failed | run `python dev/tools/lbr_deploy.py build` |
| `cannot open <port>` | wrong port, cable unplugged, or the port is in use (close any serial monitor) | find the port again (section 3, step 4) |
| `the Central does not answer` | the Central is off, on old firmware, or on another port | check its power; flash it (step 4); check `--port` |
| `<node> does not answer` | the node is not powered, not on the CAN harness, its CAN pins are not set, or it still runs firmware from before CAN updates | check power and the CAN connector; if it never had section 4, do section 4 |
| `<node> has the old flash layout` | the board needs its first-time setup | section 4 for that board |
| `Nothing was changed` (phase 2) | a transfer failed; every node still runs its old firmware | run the same command again. If the same node fails twice, check the CAN wiring to it and the bus error counters (`STAT` in a serial terminal on the Central) |
| `ERR_WRONG_FW` | the image is for another kind of node: a build mix-up | run `build` again, then the deploy again |
| `ERR_IMAGE` | the image is corrupt or built for the wrong chip | run `build` again, then the deploy again |
| `ERR_SIZE` | the image does not fit the node's flash | tell the firmware lead; do not deploy |
| `ROLLED BACK` | the node booted the new firmware but it did not sync to the Central within 60 s, so it went back to the old one | the car now has a mix. Check the Central was flashed in step 4 of the same release. Then either fix and deploy again, or go back to the earlier release on every board (section 8) |
| `NO ANSWER` (phase 4) | the node did not come back within 2 minutes | switch the car off and on, run `status`. If the node still does not answer, flash it over USB (section 4) |
| the car lost power, or the USB cable came out, during phase 1 or 2 | nothing changed | run the same command again |
| the car lost power during phase 3 or 4 | each node is on either its old or its new firmware | power on, run `status`, then run the deploy command again: nodes already on the new build are skipped |
| two wheel hubs claim the same corner, or one corner is missing | a hub was USB-flashed with the wrong corner target | USB-flash that hub with the target matching where it is mounted (section 4) |

## 7. Updating the cockpit display

The cockpit is not on CAN, and it is never flashed over WiFi. Update it only when a release changes
it.

1. Take the cockpit display out (or open it) so you can reach its USB port.
2. Plug the laptop into the cockpit's USB port and find the port.
3. Run:

   ```sh
   python dev/tools/lbr_deploy.py usb cockpit --port /dev/ttyUSB0
   ```

   It must end with `DONE: cockpit now runs ...`.
4. Put it back, power the car, and check the cockpit shows live data again within a few seconds.

## 8. Going back to an earlier release

If a release misbehaves, put the last good release on every board:

```sh
git checkout <last good release>
python dev/tools/lbr_deploy.py build
python dev/tools/lbr_deploy.py usb central --port /dev/ttyUSB0
python dev/tools/lbr_deploy.py can --port /dev/ttyUSB0
python dev/tools/lbr_deploy.py status --port /dev/ttyUSB0
```

This is the same as a routine deploy (section 5) with an older commit. The tool compares the build
with what each node runs, so every node that differs gets the older firmware.

## 9. How it works

- Every sensor node has two firmware slots. The running firmware is never overwritten: a new image
  goes into the other slot.
- The laptop sends each image to the Central over USB in small pieces; the Central puts each piece
  on the bus and waits for the node to confirm it. A lost piece is sent again. At the end the node
  checks the whole image (CRC-32, SHA-256, right chip, right kind of node) before accepting it.
- Only when every node has a checked image does the tool tell them all to switch. They reboot
  together, so the car never runs a mix of releases for longer than a reboot.
- After switching, each node must sync to the Central's clock within 60 seconds, which proves its
  CAN works. Then it confirms the new firmware. If it crashes or never syncs, the bootloader goes
  back to the old firmware on its own. A wheel hub that received another corner's firmware notices
  on its first boot and goes back at once.
- Update traffic is recorded in the Central's SD log like every other frame.

Wire format: [`can-ids.md`](can-ids.md), "Firmware update". Code: `car/components/update`
(node side and Central relay) and `dev/tools/lbr_deploy.py` (laptop side).
