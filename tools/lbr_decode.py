#!/usr/bin/env python3
"""Reference decoder for LBR logs, CAN captures, and live link streams.

This is an independent implementation of the formats in docs/protocol.md,
written from the spec rather than from the firmware source. It serves as
  - a bench tool: read a session log, decode a captured USB/WiFi stream, or
    talk to the gateway over USB serial (needs pyserial);
  - a cross-check: --selftest decodes every golden fixture in tests/fixtures
    and compares the result with the expected JSON the firmware produced.
    lbr-dashboard should pass the same fixtures with its own decoder.

The decoder is schema-driven: names, value types, units, events, and
reasons come from catalog.json, never from tables in this file.

Usage:
  lbr_decode.py log FILE.lbr [--json]
  lbr_decode.py stream FILE [--json]
  lbr_decode.py serial PORT [--baud N] [--cmd "STAT"|"LIST"|"DUMP name"] [--out FILE]
  lbr_decode.py --selftest [FIXTURES_DIR]
"""

import argparse
import json
import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FIXTURES = os.path.join(HERE, "..", "tests", "fixtures")

LOG_SYNC = b"\xa5\x5a"
LOG_HEADER = 6
LOG_TRAILER = 4
LOG_MAX_PAYLOAD = 2048
SESSION_FIXED = 32

PKT_HEADER = 16
PKT_CRC = 4
PKT_VERSION = 1

CAN_KIND_SAMPLE = 0
CAN_KIND_SYSTEM = 1
SYS_TIME_SYNC = 0x0001
SYS_TIME_FOLLOW_UP = 0x0002
SYS_HEALTH_BASE = 0x0100
HEALTH_PAGES = 12

REC_NAMES = {1: "SESSION", 2: "SAMPLE", 3: "HEALTH", 4: "CAN_FRAME", 5: "LOSS", 6: "TEXT",
             7: "END"}
REC_SAMPLE, REC_HEALTH, REC_CAN_FRAME, REC_TEXT = 2, 3, 4, 6
REC_FILE_INFO, REC_FILE_CHUNK, REC_FILE_END = 0x10, 0x11, 0x12


def crc32(data, crc=0):
    return zlib.crc32(data, crc) & 0xFFFFFFFF


class Catalog:
    """Signal metadata loaded from catalog.json (the shared schema)."""

    def __init__(self, path):
        with open(path) as f:
            self.raw = json.load(f)
        self.signals = {s["id"]: s for s in self.raw["signals"]}
        self.types = self.raw["value_types"]
        self.reasons = self.raw["unavailable_reasons"]
        self.events = {e["code"]: e["name"] for e in self.raw["events"]}
        self.event_ids = {s["id"] for s in self.raw["signals"] if s["name"] in ("EVENT", "FAULT")}

    def computed_hash(self):
        crc = 0
        for s in self.raw["signals"]:
            rec = struct.pack("<HBBB", s["id"], self.types.index(s["type"]),
                              self.raw["units"].index(s["unit"]), s["can_prio"])
            crc = crc32(rec, crc)
            crc = crc32(s["name"].encode("ascii") + b"\0", crc)
        return crc

    def sample_dict(self, signal, source, type_name, valid, synced, seq_short, seq, t_us, bits):
        info = self.signals.get(signal)
        d = {"signal": signal, "name": info["name"] if info else "?", "source": source,
             "type": type_name, "valid": valid, "synced": synced, "seq_short": seq_short,
             "seq": seq, "t_us": t_us, "value_bits": "%08x" % bits}
        if not valid:
            d["reason"] = self.reasons[bits] if bits < len(self.reasons) else "?"
        elif signal in self.event_ids:
            d["event"] = self.events.get(bits & 0xFFFF, "?")
            d["event_arg"] = bits >> 16
        return d

    def value(self, d):
        """Human-readable value of a decoded sample dict."""
        if not d["valid"]:
            return "n/a (%s)" % d["reason"]
        bits = int(d["value_bits"], 16)
        t = d["type"]
        if t == "f32":
            return "%.6g" % struct.unpack("<f", struct.pack("<I", bits))[0]
        if t == "i32":
            return str(struct.unpack("<i", struct.pack("<I", bits))[0])
        if t == "bool":
            return "true" if bits else "false"
        if "event" in d:
            return "%s(%d)" % (d["event"], d["event_arg"])
        return str(bits)


# --------------------------------------------------------------------------
# Records


def decode_sample(cat, b):
    signal, source, tf, seq, t_us, bits = struct.unpack_from("<HBBIQI", b)
    if tf & 0x80 or (tf & 0x0F) >= len(cat.types):
        raise ValueError("bad sample flags")
    return cat.sample_dict(signal, source, cat.types[tf & 0x0F], bool(tf & 0x10),
                           bool(tf & 0x20), bool(tf & 0x40), seq, t_us, bits)


HEALTH_FIELDS = [
    # (page, layout, names)
    (0, "<IBBH", ["uptime_ms", "reset_reason", "state", "flags"]),
    (1, "<IHH", ["fw_version", "protocol_version", "boot_count"]),
    (2, "<II", ["catalog_hash", "config_crc"]),
    (3, "<BBHI", ["can_tec", "can_rec", None, "can_bus_off_count"]),
    (4, "<II", ["can_tx_dropped", "can_rx_dropped"]),
    (5, "<II", ["overflow_count", "queue_high_water"]),
    (6, "<II", ["write_error_count", "transport_error_count"]),
    (7, "<II", ["sensor_status", "calibration_state"]),
    (8, "<II", ["last_sample_age_ms", "last_sd_write_age_ms"]),
    (9, "<HHHH", ["acquire_hz", "publish_hz", "log_hz", None]),
    (10, "<Ii", ["time_step_count", "last_sync_error_us"]),
    (11, "<II", ["rejected_count", "can_bus_errors"]),
]


def decode_health_page(page, b, out):
    _, layout, names = HEALTH_FIELDS[page]
    for name, v in zip(names, struct.unpack(layout, b)):
        if name is None:
            if v != 0:
                raise ValueError("reserved health field set")
        else:
            out[name] = v


def decode_health(b):
    h = {"node_id": b[0]}
    for page in range(HEALTH_PAGES):
        decode_health_page(page, b[1 + 8 * page:9 + 8 * page], h)
    return h


def unwrap24(t_lo, ref):
    d = (ref - t_lo) & 0xFFFFFF
    if d >= 0x800000:
        d -= 0x1000000
    return max(ref - d, 0)


def decode_can_frame(cat, t_us, flags, can_id, dlc, data):
    prio, kind, source, sub = (can_id >> 26) & 7, (can_id >> 24) & 3, (can_id >> 16) & 0xFF, can_id & 0xFFFF
    d = {"t_us": t_us, "flags": flags, "id": can_id, "prio": prio, "kind": kind, "source": source,
         "sub": sub, "dlc": dlc, "data": data[:dlc].hex()}
    if kind == CAN_KIND_SAMPLE and dlc == 8 and source != 0:
        b7 = data[7]
        info = cat.signals.get(sub)
        bits = struct.unpack_from("<I", data)[0]
        t_lo = data[4] | data[5] << 8 | data[6] << 16
        d["sample"] = cat.sample_dict(sub, source, info["type"] if info else "raw",
                                      bool(b7 & 0x80), bool(b7 & 0x40), True, b7 & 0x3F,
                                      unwrap24(t_lo, t_us), bits)
    elif kind == CAN_KIND_SYSTEM and sub == SYS_TIME_SYNC and dlc >= 3:
        d["time_sync"] = {"seq": data[0], "epoch": data[1] | data[2] << 8}
    elif kind == CAN_KIND_SYSTEM and sub == SYS_TIME_FOLLOW_UP and dlc == 8:
        d["time_follow_up"] = {"seq": data[0], "tx_us": struct.unpack_from("<Q", data)[0] >> 8}
    elif kind == CAN_KIND_SYSTEM and SYS_HEALTH_BASE <= sub < SYS_HEALTH_BASE + HEALTH_PAGES and dlc == 8:
        try:
            decode_health_page(sub - SYS_HEALTH_BASE, data, {})
            d["health_page"] = sub - SYS_HEALTH_BASE
        except ValueError:
            pass
    return d


def decode_can_record(cat, b):
    t_us, can_id, flags, dlc = struct.unpack_from("<QIBB", b)
    if dlc > 8:
        raise ValueError("bad dlc")
    return decode_can_frame(cat, t_us, flags, can_id, dlc, b[14:22])


# --------------------------------------------------------------------------
# Log files


class LogReader:
    """Record scanner with the spec's recovery rules: skip garbage to the next
    valid record; an incomplete record at the end is a truncated tail."""

    def __init__(self, data):
        self.data, self.pos = data, 0
        self.corrupt_regions = self.skipped_bytes = self.truncated_tail = 0

    def record_at(self, pos):
        d = self.data
        left = len(d) - pos
        if left < LOG_HEADER + LOG_TRAILER or d[pos:pos + 2] != LOG_SYNC or d[pos + 3] != 0:
            return None
        plen = d[pos + 4] | d[pos + 5] << 8
        n = LOG_HEADER + plen + LOG_TRAILER
        if plen > LOG_MAX_PAYLOAD or left < n:
            return None
        if struct.unpack_from("<I", d, pos + LOG_HEADER + plen)[0] != crc32(d[pos + 2:pos + LOG_HEADER + plen]):
            return None
        return d[pos + 2], d[pos + LOG_HEADER:pos + LOG_HEADER + plen], n

    def tail_candidate(self, pos):
        d = self.data
        left = len(d) - pos
        if d[pos] != LOG_SYNC[0] or (left >= 2 and d[pos + 1] != LOG_SYNC[1]):
            return False
        if left < LOG_HEADER:
            return True
        plen = d[pos + 4] | d[pos + 5] << 8
        return plen <= LOG_MAX_PAYLOAD and left < LOG_HEADER + plen + LOG_TRAILER

    def __iter__(self):
        while True:
            in_garbage = False
            while self.pos < len(self.data):
                rec = self.record_at(self.pos)
                if rec:
                    offset = self.pos
                    self.pos += rec[2]
                    yield offset, rec[0], rec[1]
                    break
                if self.tail_candidate(self.pos) and not any(
                        self.record_at(q) for q in range(self.pos + 1, len(self.data))):
                    self.truncated_tail = len(self.data) - self.pos
                    self.pos = len(self.data)
                    return
                if not in_garbage:
                    self.corrupt_regions += 1
                    in_garbage = True
                self.skipped_bytes += 1
                self.pos += 1
            else:
                return


def decode_log_record(cat, offset, rtype, p):
    d = {"offset": offset, "type": REC_NAMES.get(rtype, "?")}
    if rtype == 1:
        (fmt, proto, fw, chash, ccrc, node, reset, _, boots, start) = struct.unpack_from("<HHIIIBBHIQ", p)
        d.update({"format_version": fmt, "protocol_version": proto, "fw_version": fw,
                  "catalog_hash": chash, "config_crc": ccrc, "node_id": node,
                  "reset_reason": reset, "boot_count": boots, "start_time_us": start,
                  "config_text": p[SESSION_FIXED:].decode("latin-1")})
    elif rtype == REC_SAMPLE:
        d["sample"] = decode_sample(cat, p)
    elif rtype == REC_HEALTH:
        d["health"] = decode_health(p)
    elif rtype == REC_CAN_FRAME:
        d["frame"] = decode_can_record(cat, p)
    elif rtype == 5:
        recs, nbytes, t = struct.unpack_from("<IIQ", p)
        d.update({"dropped_records": recs, "dropped_bytes": nbytes, "t_us": t})
    elif rtype == REC_TEXT:
        d["text"] = p.decode("latin-1")
    elif rtype == 7:
        t, n = struct.unpack_from("<QI", p)
        d.update({"t_us": t, "records_written": n})
    return d


def decode_log(cat, data):
    r = LogReader(data)
    records = [decode_log_record(cat, off, t, p) for off, t, p in r]
    return {"records": records, "truncated_tail": r.truncated_tail,
            "corrupt_regions": r.corrupt_regions, "skipped_bytes": r.skipped_bytes}


# --------------------------------------------------------------------------
# Link streams


def cobs_decode(b):
    out, i = bytearray(), 0
    while i < len(b):
        code = b[i]
        i += 1
        if code == 0 or i + code - 1 > len(b):
            raise ValueError("bad COBS")
        out += b[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(b):
            out.append(0)
    return bytes(out)


def parse_packet(pkt):
    if len(pkt) < PKT_HEADER + PKT_CRC or pkt[0] != PKT_VERSION:
        raise ValueError("bad packet header")
    body = len(pkt) - PKT_CRC
    if struct.unpack_from("<I", pkt, body)[0] != crc32(pkt[:body]):
        raise ValueError("bad packet CRC")
    records, pos = [], PKT_HEADER
    while pos < body:
        if pos + 2 > body or pos + 2 + pkt[pos + 1] > body:
            raise ValueError("bad record framing")
        records.append((pkt[pos], pkt[pos + 2:pos + 2 + pkt[pos + 1]]))
        pos += 2 + pkt[pos + 1]
    if len(records) != pkt[3]:
        raise ValueError("record count mismatch")
    link, source = pkt[1], pkt[2]
    seq, t_us = struct.unpack_from("<IQ", pkt, 4)
    return {"link": link, "source": source, "seq": seq, "t_us": t_us}, records


def iter_frames(data):
    """Split a byte stream at 0x00 delimiters (empty frames are skipped)."""
    start = 0
    for i, c in enumerate(data):
        if c == 0:
            if i > start:
                yield data[start:i]
            start = i + 1


def decode_packet_records(cat, records):
    out = []
    for rtype, p in records:
        d = {"type": rtype, "len": len(p)}
        if rtype == REC_CAN_FRAME:
            d["frame"] = decode_can_record(cat, p)
        elif rtype == REC_TEXT:
            d["text"] = p.decode("latin-1")
        out.append(d)
    return out


def decode_stream(cat, data):
    packets, bad = [], 0
    for frame in iter_frames(data):
        try:
            hdr, records = parse_packet(cobs_decode(frame))
        except ValueError:
            bad += 1
            continue
        hdr["records"] = decode_packet_records(cat, records)
        packets.append(hdr)
    return {"packets": packets, "bad_frames": bad}


# --------------------------------------------------------------------------
# Self test against the golden fixtures


def selftest(fixtures):
    cat = Catalog(os.path.join(fixtures, "catalog.json"))
    checks = []

    def read(name):
        with open(os.path.join(fixtures, name), "rb") as f:
            return f.read()

    def expect(name):
        with open(os.path.join(fixtures, name)) as f:
            return json.load(f)

    checks.append(("catalog hash", cat.computed_hash() == cat.raw["catalog_hash"]))

    data = read("samples.bin")
    got = {"record_size": 20, "samples": [decode_sample(cat, data[i:i + 20])
                                          for i in range(0, len(data), 20)]}
    checks.append(("samples", got == expect("samples.json")))

    got = {"record_size": 97, "health": decode_health(read("health.bin"))}
    checks.append(("health", got == expect("health.json")))

    data = read("can_capture.bin")
    got = {"record_size": 22, "frames": [decode_can_record(cat, data[i:i + 22])
                                         for i in range(0, len(data), 22)]}
    checks.append(("can capture", got == expect("can_capture.json")))

    checks.append(("session log", decode_log(cat, read("session.lbr")) == expect("session.json")))
    checks.append(("usb stream", decode_stream(cat, read("stream_usb.bin")) == expect("stream_usb.json")))

    for name, ok in checks:
        print("%s %s" % ("PASS" if ok else "FAIL", name))
    return 0 if all(ok for _, ok in checks) else 1


# --------------------------------------------------------------------------
# CLI


def print_sample(cat, d, prefix=""):
    print("%s%12d us  %-14s %-18s = %s%s" % (
        prefix, d["t_us"], "node %d" % d["source"], d["name"], cat.value(d),
        "" if d["synced"] else "  [unsynced time]"))


def cmd_log(cat, path, as_json):
    with open(path, "rb") as f:
        out = decode_log(cat, f.read())
    if as_json:
        json.dump(out, sys.stdout, indent=1)
        return
    for r in out["records"]:
        if r["type"] == "SAMPLE":
            print_sample(cat, r["sample"])
        elif r["type"] == "CAN_FRAME" and "sample" in r["frame"]:
            print_sample(cat, r["frame"]["sample"], "can ")
        elif r["type"] == "TEXT":
            print("text  %s" % r["text"])
        elif r["type"] != "CAN_FRAME":
            print({k: v for k, v in r.items() if k != "offset"})
    print("-- %d records, %d corrupt regions (%d bytes skipped), truncated tail %d bytes" % (
        len(out["records"]), out["corrupt_regions"], out["skipped_bytes"], out["truncated_tail"]))


def cmd_stream(cat, path, as_json):
    with open(path, "rb") as f:
        out = decode_stream(cat, f.read())
    json.dump(out, sys.stdout, indent=1) if as_json else print(
        "%d packets, %d bad frames" % (len(out["packets"]), out["bad_frames"]))


def cmd_serial(cat, port, baud, cmd, out_path):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("serial mode needs pyserial: pip install pyserial")
    link = serial.Serial(port, baud, timeout=0.2)
    if cmd:
        link.write((cmd + "\n").encode("ascii"))
    out = open(out_path, "wb") if out_path else None
    buf, last_seq = bytearray(), None
    try:
        while True:
            buf += link.read(4096)
            *frames, rest = bytes(buf).split(b"\0")
            buf = bytearray(rest)
            for frame in frames:
                if not frame:
                    continue
                try:
                    hdr, records = parse_packet(cobs_decode(frame))
                except ValueError:
                    continue  # boot ROM text, panic output, or line noise
                if last_seq is not None and (hdr["seq"] - last_seq) & 0xFFFFFFFF != 1:
                    print("!! %d packet(s) lost on the link" % (((hdr["seq"] - last_seq) & 0xFFFFFFFF) - 1))
                last_seq = hdr["seq"]
                for rtype, p in records:
                    if rtype == REC_TEXT:
                        print(p.decode("latin-1"))
                    elif rtype == REC_CAN_FRAME:
                        d = decode_can_record(cat, p)
                        if "sample" in d:
                            print_sample(cat, d["sample"])
                    elif rtype == REC_FILE_INFO:
                        print("%-14s %10d bytes" % (p[4:].decode("ascii"), struct.unpack_from("<I", p)[0]))
                    elif rtype == REC_FILE_CHUNK and out:
                        out.seek(struct.unpack_from("<I", p)[0])
                        out.write(p[4:])
                    elif rtype == REC_FILE_END:
                        total, crc = struct.unpack_from("<II", p)
                        print("dump complete: %d bytes, crc %08x" % (total, crc))
                        if out:
                            out.close()
                            return
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", nargs="?", const=DEFAULT_FIXTURES, metavar="FIXTURES_DIR")
    ap.add_argument("--catalog", default=os.path.join(DEFAULT_FIXTURES, "catalog.json"))
    ap.add_argument("mode", nargs="?", choices=["log", "stream", "serial"])
    ap.add_argument("target", nargs="?")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--cmd")
    ap.add_argument("--out")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest(args.selftest))
    if not args.mode or not args.target:
        ap.error("need a mode and a target (or --selftest)")
    cat = Catalog(args.catalog)
    if args.mode == "log":
        cmd_log(cat, args.target, args.json)
    elif args.mode == "stream":
        cmd_stream(cat, args.target, args.json)
    else:
        cmd_serial(cat, args.target, args.baud, args.cmd, args.out)


if __name__ == "__main__":
    main()
