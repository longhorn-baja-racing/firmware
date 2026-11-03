#!/usr/bin/env python3
"""Build and deploy LBR firmware. Step-by-step guide: docs/deploying.md.

  central, cockpit         USB cable to that board        (lbr_deploy.py usb ...)
  the nine CAN nodes       over CAN, through the Central  (lbr_deploy.py can ...)
  any board, first time    USB cable to that board        (lbr_deploy.py usb ...)

Commands:
  lbr_deploy.py list
      every target, its board, and how it is updated
  lbr_deploy.py build [TARGET ...]
      build images (default: all targets) with each node's own sdkconfig
  lbr_deploy.py menuconfig TARGET
      edit a target's settings (pins, inputs) in the config its builds use
  lbr_deploy.py usb TARGET --port PORT
      build TARGET and flash it over a USB cable plugged into that board
  lbr_deploy.py status --port PORT
      ask every CAN node (through the Central) what it runs vs. the local builds
  lbr_deploy.py can --port PORT [TARGET ...] [--force] [--yes] [--skip-missing]
      update CAN nodes through the Central: stage every image first, then
      switch all of them together, then check each one confirmed itself

ESP-IDF must be active (. ~/.espressif/tools/activate_idf_v6.1.sh) for build,
menuconfig, and usb. status and can need pyserial (pip install pyserial).
"""

import argparse
import base64
import hashlib
import importlib.util
import os
import struct
import subprocess
import sys
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
NODES = os.path.join(ROOT, "car", "nodes")

# target -> (app directory, corner position or None, chip, how it is updated)
TARGETS = {
    "central":      ("central", None, "ESP32-C5", "usb"),
    "imu":          ("imu", None, "ESP32-C5", "can"),
    "powertrain":   ("powertrain", None, "ESP32", "can"),
    "driver_input": ("driver_input", None, "ESP32", "can"),
    "temperature":  ("temperature", None, "ESP32", "can"),
    "stress":       ("stress", None, "ESP32", "can"),
    "corner_fl":    ("corner", "fl", "Mini ESP32-C5", "can"),
    "corner_fr":    ("corner", "fr", "Mini ESP32-C5", "can"),
    "corner_bl":    ("corner", "bl", "Mini ESP32-C5", "can"),
    "corner_br":    ("corner", "br", "Mini ESP32-C5", "can"),
    "cockpit":      ("cockpit", None, "ESP32-C5", "usb"),
}
CAN_TARGETS = [t for t, v in TARGETS.items() if v[3] == "can"]

CHUNK = 448        # image bytes per DATA line (LBR_UPD_LINE_DATA_MAX)
FRAME_BYTES = 7    # image bytes per CAN frame
CONFIRM_WAIT_S = 120  # node reboot + sync + confirm (it rolls back after 60 s)


def die(msg):
    sys.exit("\nERROR: " + msg)


# --------------------------------------------------------------------------
# Builds


def target_paths(target):
    app, pos, _, _ = TARGETS[target]
    proj = os.path.join(NODES, app)
    if pos is None:
        return proj, os.path.join(proj, "build"), None, None
    return (proj, os.path.join(proj, "build-" + target), os.path.join(proj, "sdkconfig." + target),
            os.path.join(proj, "positions", pos + ".defaults"))


def idf_cmd(target, *action):
    if not os.environ.get("IDF_PATH"):
        die("ESP-IDF is not active. Run:  . ~/.espressif/tools/activate_idf_v6.1.sh")
    proj, build, sdkconfig, overlay = target_paths(target)
    cmd = [sys.executable, os.path.join(os.environ["IDF_PATH"], "tools", "idf.py"), "-C", proj,
           "-B", build]
    if sdkconfig:
        cmd += ["-DSDKCONFIG=" + sdkconfig, "-DLBR_SDKCONFIG_EXTRA=" + overlay]
    return cmd + list(action)


def run(cmd):
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def check_corner_config(target):
    """The position in a corner's config must match the target, or the board
    would come up as a different wheel."""
    _, _, sdkconfig, _ = target_paths(target)
    pos = TARGETS[target][1].upper()
    try:
        text = open(sdkconfig).read()
    except OSError:
        return
    if "CONFIG_LBR_CORNER_%s=y" % pos not in text:
        die("%s is not set to position %s. Run: lbr_deploy.py menuconfig %s, choose %s under"
            " 'LBR corner node', and build again." % (sdkconfig, pos, target, pos))


def build(target):
    print("\n=== build %s ===" % target)
    if run(idf_cmd(target, "build")) != 0:
        die("build of %s failed (see above)." % target)
    if TARGETS[target][1]:
        check_corner_config(target)


# --------------------------------------------------------------------------
# Images


class Image:
    """An app image as built (.bin): what it is, and the hash the node reports."""

    def __init__(self, target):
        _, build_dir, _, _ = target_paths(target)
        app = TARGETS[target][0]
        self.target = target
        self.path = os.path.join(build_dir, "lbr_%s.bin" % app)
        if not os.path.exists(self.path):
            die("no build of %s (%s). Run: lbr_deploy.py build %s" % (target, self.path, target))
        if TARGETS[target][1]:
            check_corner_config(target)
        self.data = open(self.path, "rb").read()
        d = self.data
        if len(d) < 200 or d[0] != 0xE9:
            die("%s is not an ESP app image" % self.path)
        magic, = struct.unpack_from("<I", d, 32)
        if magic != 0xABCD5432:
            die("%s has no app description" % self.path)
        text = lambda off, n: d[off:off + n].split(b"\0")[0].decode("ascii", "replace")
        self.version = text(48, 32)
        self.project = text(80, 32)
        self.built = text(112, 16) + " " + text(128, 16)
        if self.project != "lbr_" + app:
            die("%s is project %s, expected lbr_%s" % (self.path, self.project, app))
        if not d[23] or hashlib.sha256(d[:-32]).digest() != d[-32:]:
            die("%s has no valid appended SHA-256 (rebuild it)" % self.path)
        self.hash = d[-32:-28].hex()
        self.crc = zlib.crc32(d) & 0xFFFFFFFF
        self.mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(self.path)))


# --------------------------------------------------------------------------
# The Central over USB


def load_decoder():
    spec = importlib.util.spec_from_file_location("lbr_decode", os.path.join(HERE, "lbr_decode.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Central:
    """The Central's USB link: send command lines, read TEXT replies out of
    the COBS-framed packet stream."""

    def __init__(self, port, baud):
        try:
            import serial  # pyserial
        except ImportError:
            die("this command needs pyserial: pip install pyserial")
        self.dec = load_decoder()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            die("cannot open %s: %s" % (port, e))
        self.buf = b""

    def texts(self, deadline):
        while time.monotonic() < deadline:
            self.buf += self.ser.read(4096)
            *frames, self.buf = self.buf.split(b"\0")
            for frame in frames:
                if not frame:
                    continue
                try:
                    _, records = self.dec.parse_packet(self.dec.cobs_decode(frame))
                except ValueError:
                    continue
                for rtype, p in records:
                    if rtype == self.dec.REC_TEXT:
                        yield p.decode("latin-1")

    def upd(self, line, timeout=25.0):
        """One UPD step; returns the reply fields, or None if the Central said nothing."""
        self.ser.reset_input_buffer()
        self.buf = b""
        self.ser.write(("UPD " + line + "\n").encode("ascii"))
        who, op = line.split(" ")[:2]
        for text in self.texts(time.monotonic() + timeout):
            if text.startswith("UPD ERR"):
                die("the Central rejected '%s': %s" % (line[:60], text))
            if text.startswith("UPD %s %s " % (who, op)):
                return parse_reply(text)
        return None


def parse_reply(text):
    words = text.split()
    r = {"text": text, "result": words[3] if len(words) > 3 else "?"}
    for w in words[4:]:
        if "=" in w:
            k, v = w.split("=", 1)
            r[k] = v
    return r


# --------------------------------------------------------------------------
# Steps


def info(central, target):
    r = central.upd("%s INFO" % target, timeout=5)
    if r is None:
        die("the Central does not answer. Is it powered, flashed with this firmware, and is"
            " --port right?")
    return r


def stage(central, img):
    """Send one image and have the node check it. Returns None on success,
    otherwise the reason."""
    size = len(img.data)
    r = central.upd("%s BEGIN %d" % (img.target, size))
    if r is None or r["result"] != "OK":
        return "BEGIN: %s" % (r["text"] if r else "no reply")
    k, retries, t0, last = 0, 0, time.monotonic(), -1
    while k * FRAME_BYTES < size:
        off = k * FRAME_BYTES
        chunk = img.data[off:off + CHUNK]
        r = central.upd("%s DATA %d %s" % (img.target, k, base64.b64encode(chunk).decode()),
                        timeout=8)
        if r is None or r["result"] == "TIMEOUT":
            retries += 1
            r = central.upd("%s QUERY" % img.target, timeout=8)
            if r is None or r["result"] == "TIMEOUT":
                if retries > 10:
                    return "node stopped answering at byte %d" % off
                continue
        if r.get("state") != "RECEIVING":
            return "node left the transfer at byte %d: %s" % (off, r["text"])
        nk = int(r["next"])
        if nk != k + (len(chunk) + FRAME_BYTES - 1) // FRAME_BYTES:
            retries += 1  # frames were missed: resume where the node is
            if retries > 50:
                return "too many resends at byte %d" % off
        k = nk
        pct = min(100, k * FRAME_BYTES * 100 // size)
        if pct != last:
            rate = k * FRAME_BYTES / max(time.monotonic() - t0, 0.001) / 1024
            print("\r  %-12s %3d %%  %6.1f KB/s  resends %d " % (img.target, pct, rate, retries),
                  end="", flush=True)
            last = pct
    print()
    r = central.upd("%s END %08x" % (img.target, img.crc), timeout=30)
    if r is None or r["result"] != "OK" or r.get("state") != "STAGED":
        return "END: %s" % (r["text"] if r else "no reply")
    return None


def confirm_prompt(text):
    print(text)
    if input("Type yes to continue: ").strip().lower() != "yes":
        sys.exit("Stopped. Nothing was changed.")


# --------------------------------------------------------------------------
# Commands


def cmd_list(_args):
    print("%-13s %-15s %-10s %s" % ("target", "board", "update by", "app directory"))
    for t, (app, pos, chip, how) in TARGETS.items():
        where = "car/nodes/%s" % app + (" (position %s)" % pos.upper() if pos else "")
        print("%-13s %-15s %-10s %s" % (t, chip, how.upper(), where))


def cmd_build(args):
    for t in args.targets or list(TARGETS):
        build(t)
    print("\nBuilt: " + ", ".join(args.targets or list(TARGETS)))


def cmd_menuconfig(args):
    sys.exit(run(idf_cmd(args.target, "menuconfig")))


def cmd_usb(args):
    t = args.target
    build(t)
    print("\n=== flash %s over USB on %s ===" % (t, args.port))
    if run(idf_cmd(t, "-p", args.port, "flash")) != 0:
        die("flashing failed. Check the cable is in the %s board, the port, and hold BOOT while"
            " pressing RESET if the board does not enter download mode." % t)
    img = Image(t)
    print("\nDONE: %s now runs %s (hash %s). Watch it boot with:\n  idf.py -C %s -B %s -p %s"
          " monitor" % (t, img.version, img.hash, *target_paths(t)[:2], args.port))


def cmd_status(args):
    central = Central(args.port, args.baud)
    print("%-13s %-10s %-9s %-8s %s" % ("node", "running", "confirmed", "local", ""))
    for t in CAN_TARGETS:
        r = central.upd("%s INFO" % t, timeout=5)
        try:
            local = Image(t).hash
        except SystemExit:
            local = None
        if r is None:
            die("the Central does not answer on %s" % args.port)
        if r["result"] == "TIMEOUT":
            print("%-13s %-10s" % (t, "(no answer: not on the bus?)"))
            continue
        if r["result"] == "ERR_NO_SLOT":
            print("%-13s %-10s" % (t, "(old layout: flash once over USB)"))
            continue
        same = "" if local is None else ("same as local build" if local == r["hash"]
                                         else "DIFFERS from local build %s" % local)
        print("%-13s %-10s %-9s %-8s %s" % (t, r["hash"], "no" if r["pending"] == "1" else "yes",
                                           local or "-", same))


def cmd_can(args):
    targets = args.targets or CAN_TARGETS
    for t in targets:
        if t not in CAN_TARGETS:
            die("%s is not updated over CAN; use: lbr_deploy.py usb %s --port ..." % (t, t))
    images = {t: Image(t) for t in targets}

    print("Images to deploy over CAN:")
    for t, img in images.items():
        print("  %-12s %-20s %7d bytes  hash %s  built %s" % (t, img.version, len(img.data),
                                                              img.hash, img.mtime))
    dirty = [t for t, img in images.items() if img.version.endswith("-dirty")]
    if dirty:
        print("\nWARNING: built from uncommitted changes (%s). Fine for bench tests; for the race,"
              " commit first so the version on the car can be traced." % ", ".join(dirty))
    if not args.yes:
        confirm_prompt("\nThe car must be PARKED, on a charger or a full battery, with the laptop"
                       " on the Central's USB port.\nDo not switch the car off until this"
                       " finishes.")

    central = Central(args.port, args.baud)
    print("\n[1/4] Checking every node")
    central.upd("ALL ABORT", timeout=5)  # clear anything left staged by an earlier attempt
    todo, problems = [], []
    for t in targets:
        r = info(central, t)
        if r["result"] == "TIMEOUT":
            problems.append("%s does not answer (not on the bus, no power, or CAN fault)" % t)
        elif r["result"] == "ERR_NO_SLOT":
            problems.append("%s has the old flash layout: flash it once over USB" % t)
        elif r["result"] != "OK":
            problems.append("%s: %s" % (t, r["text"]))
        elif r["hash"] == images[t].hash and not args.force:
            print("  %-12s already runs this build (%s): skipped" % (t, r["hash"]))
        else:
            print("  %-12s runs %s -> will get %s" % (t, r["hash"], images[t].hash))
            todo.append((t, r["hash"]))
    if problems:
        print("\n".join("  PROBLEM: " + p for p in problems))
        if not args.skip_missing:
            die("not every node is ready. Nothing was changed. Fix the problems above, or rerun"
                " with --skip-missing to update only the nodes that answered.")
    if not todo:
        print("\nNothing to do: every node already runs these builds.")
        return

    print("\n[2/4] Sending images (nodes keep running their current firmware)")
    for t, _ in todo:
        why = stage(central, images[t])
        if why:
            central.upd("ALL ABORT", timeout=5)
            die("%s: %s\nNothing was changed: every node still runs its old firmware. Run the"
                " same command again; see docs/deploying.md, Troubleshooting." % (t, why))
        print("  %-12s staged and checked" % t)

    print("\n[3/4] Switching %d node(s) to the new firmware together" % len(todo))
    central.upd("ALL ACTIVATE", timeout=5)

    print("\n[4/4] Waiting for each node to boot, sync, and confirm (up to %d s)" % CONFIRM_WAIT_S)
    deadline = time.monotonic() + CONFIRM_WAIT_S
    results = {}
    time.sleep(5)  # ACTIVATE reboots each node within a second; after this, an old hash is final
    while time.monotonic() < deadline and len(results) < len(todo):
        for t, old in todo:
            if t in results:
                continue
            r = central.upd("%s INFO" % t, timeout=3)
            if r is None or r["result"] != "OK":
                continue  # still booting
            if r["hash"] == images[t].hash and r["pending"] == "0":
                results[t] = "OK"
                print("  %-12s OK, running %s and confirmed" % (t, r["hash"]))
            elif r["hash"] == old and r["pending"] == "0":
                results[t] = "ROLLED BACK"
                print("  %-12s ROLLED BACK to %s" % (t, old))
        time.sleep(2)

    print("\nResult:")
    ok = True
    for t, old in todo:
        res = results.get(t, "NO ANSWER")
        ok = ok and res == "OK"
        print("  %-12s %s" % (t, res))
    if not ok:
        die("not every node took the update. ROLLED BACK = the new firmware did not sync to the"
            " Central within 60 s, so the node went back to its old firmware. See"
            " docs/deploying.md, Troubleshooting.")
    print("\nDEPLOY COMPLETE: every node runs the new firmware.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("build")
    p.add_argument("targets", nargs="*", metavar="TARGET")
    p = sub.add_parser("menuconfig")
    p.add_argument("target", choices=list(TARGETS))
    p = sub.add_parser("usb")
    p.add_argument("target", choices=list(TARGETS))
    p.add_argument("--port", required=True)
    for name in ("status", "can"):
        p = sub.add_parser(name)
        p.add_argument("--port", required=True)
        p.add_argument("--baud", type=int, default=921600)
        if name == "can":
            p.add_argument("targets", nargs="*", metavar="TARGET")
            p.add_argument("--force", action="store_true", help="send even if already up to date")
            p.add_argument("--yes", action="store_true", help="skip the parked-car prompt")
            p.add_argument("--skip-missing", action="store_true",
                           help="update the nodes that answer; leave the others")
    args = ap.parse_args()
    for t in getattr(args, "targets", None) or []:
        if t not in TARGETS:
            ap.error("unknown target %s (see: lbr_deploy.py list)" % t)
    {"list": cmd_list, "build": cmd_build, "menuconfig": cmd_menuconfig, "usb": cmd_usb,
     "status": cmd_status, "can": cmd_can}[args.cmd](args)


if __name__ == "__main__":
    main()
