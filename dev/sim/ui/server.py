#!/usr/bin/env python3
"""Browser front end for the LBR car simulator (development only).

Starts dev/sim's lbr_sim in interactive mode, paces it against the wall
clock (pause, step, slow motion, fast forward), plays scenario files, and
serves a 2D view of the car at http://127.0.0.1:8765/.

    cmake -S dev -B build-dev && cmake --build build-dev
    python3 dev/sim/ui/server.py            # then open the printed URL

Standard library only. The simulator speaks JSON lines on stdout and takes
one command per line on stdin; this script only relays and paces them.
"""

import argparse
import glob
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCENARIOS = os.path.join(HERE, "..", "scenarios")
TICK_S = 0.05  # wall-clock pacing period
MAX_TRACE_PER_BATCH = 80


def finite(text):
    v = float(text)
    if not math.isfinite(v):
        raise ValueError("not a finite number: %s" % text)
    return v


def find_sim(explicit):
    if explicit:
        return explicit
    exe = "lbr_sim.exe" if os.name == "nt" else "lbr_sim"
    for pattern in ("build-dev*/sim/" + exe, "build*/sim/" + exe, "dev/build*/sim/" + exe):
        hits = sorted(glob.glob(os.path.join(REPO, pattern)), key=os.path.getmtime, reverse=True)
        if hits:
            return hits[0]
    sys.exit("lbr_sim not found: build it with 'cmake -S dev -B build-dev && "
             "cmake --build build-dev', or pass --sim PATH")


class Sim:
    """One lbr_sim process plus the pacing state around it."""

    def __init__(self, exe, out_dir, seed):
        self.exe, self.out_dir, self.seed = exe, out_dir, seed
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.clients = []
        self.clients_lock = threading.Lock()
        self.speed = 1.0
        self.paused = False
        self.step_ms = 0.0
        self.script = []        # [(t_seconds, command)]
        self.script_name = ""
        self.t = 0.0
        self.last_snap = None
        self.proc = None
        self.start()

    # -- process -----------------------------------------------------------
    def start(self):
        if self.proc is not None:
            try:
                self.proc.stdin.write("quit\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()
        os.makedirs(self.out_dir, exist_ok=True)
        self.proc = subprocess.Popen(
            [self.exe, "--interactive", "--seed", str(self.seed), "--out", self.out_dir],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.t = 0.0
        self.script, self.script_name = [], ""
        shown = (os.path.relpath(self.out_dir, REPO)
                 if self.out_dir.startswith(REPO + os.sep) else self.out_dir)
        self.broadcast([{"ev": "ui", "msg": "simulator started (seed %d, logs in %s)"
                         % (self.seed, shown)}])
        self.exchange(None)  # initial snapshot

    def exchange(self, line):
        """Send one command (or nothing) and collect the reply up to 'done'."""
        if line is not None:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        batch, traces = [], 0
        while True:
            raw = self.proc.stdout.readline()
            if not raw:
                self.broadcast([{"ev": "error", "msg": "simulator exited"}])
                time.sleep(1)
                return
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            ev = msg.get("ev")
            if ev == "done":
                break
            if ev == "snap":
                self.t = msg["t"]
                self.last_snap = msg
            if ev == "cmd":
                traces += 1
                if traces > MAX_TRACE_PER_BATCH:
                    continue
            batch.append(msg)
        if traces > MAX_TRACE_PER_BATCH:
            batch.append({"ev": "ui", "msg": "%d command lines not shown (trace one node, "
                                              "or slow down)" % (traces - MAX_TRACE_PER_BATCH)})
        self.broadcast(batch)

    # -- clients -----------------------------------------------------------
    def broadcast(self, batch):
        if not batch:
            return
        data = "data: " + json.dumps(batch, separators=(",", ":")) + "\n\n"
        with self.clients_lock:
            for q in list(self.clients):
                if q.qsize() < 200:
                    q.put(data)

    def state(self):
        return {"ev": "ui_state", "speed": self.speed, "paused": self.paused,
                "seed": self.seed, "script": self.script_name,
                "script_left": len(self.script),
                "scenarios": sorted(os.path.splitext(os.path.basename(p))[0]
                                    for p in glob.glob(os.path.join(SCENARIOS, "*.sim")))}

    # -- control -----------------------------------------------------------
    def ui_command(self, text):
        # A bad command is reported, never allowed to stop the clock thread.
        try:
            self.ui_apply(text.split())
        except (ValueError, OSError) as e:
            self.broadcast([{"ev": "ui", "msg": "ui %s: %s" % (text.strip(), e)}])
        self.broadcast([self.state()])

    def ui_apply(self, parts):
        if not parts:
            return
        if parts[0] == "pause":
            self.paused = True
        elif parts[0] == "resume":
            self.paused = False
        elif parts[0] == "speed" and len(parts) > 1:
            self.speed = max(0.01, min(50.0, finite(parts[1])))
        elif parts[0] == "step":
            self.step_ms = max(0.0, min(60000.0, finite(parts[1]))) if len(parts) > 1 else 100.0
        elif parts[0] == "restart":
            if len(parts) > 1:
                self.seed = int(parts[1])
            self.start()
        elif parts[0] == "scenario" and len(parts) > 1:
            self.load_script(parts[1])

    def load_script(self, name):
        path = os.path.join(SCENARIOS, os.path.basename(name) + ".sim")
        steps, end = [], None
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("end "):
                    end = finite(line.split()[1])
                    continue
                fields = line.split(None, 2)
                if len(fields) != 3 or fields[0] != "at":
                    raise ValueError("bad scenario line: %s" % line)
                steps.append((finite(fields[1]), fields[2]))
        # Scenarios are written from t=0: restart so their times line up.
        self.start()
        self.script = steps + ([(end, "status")] if end is not None else [])
        self.script_name = name
        self.paused = False
        self.broadcast([{"ev": "ui", "msg": "playing scenario %s (%d steps)" % (name, len(steps))}])

    def run(self):
        while True:
            started = time.monotonic()
            while not self.commands.empty():
                text = self.commands.get()
                if text.startswith("ui "):
                    self.ui_command(text[3:])
                else:
                    self.exchange(text)
            run_ms = 0.0
            if self.step_ms > 0:
                run_ms, self.step_ms = self.step_ms, 0.0
            elif not self.paused:
                run_ms = TICK_S * 1000.0 * self.speed
            # Stop exactly at the next scenario step.
            while run_ms > 0 and self.script:
                due_ms = (self.script[0][0] - self.t) * 1000.0
                if due_ms > run_ms:
                    break
                if due_ms > 0:
                    self.exchange("run %.3f" % due_ms)
                    run_ms -= due_ms
                t, cmd = self.script.pop(0)
                self.exchange(cmd)
                if not self.script:
                    self.broadcast([{"ev": "ui", "msg": "scenario %s finished" % self.script_name},
                                    self.state()])
            if run_ms > 0:
                self.exchange("run %.3f" % run_ms)
            time.sleep(max(0.0, TICK_S - (time.monotonic() - started)))


def make_handler(sim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q = queue.Queue()
                with sim.clients_lock:
                    sim.clients.append(q)
                try:
                    first = [sim.state()] + ([sim.last_snap] if sim.last_snap else [])
                    self.wfile.write(("data: " + json.dumps(first) + "\n\n").encode())
                    self.wfile.flush()
                    while True:
                        self.wfile.write(q.get().encode())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with sim.clients_lock:
                        sim.clients.remove(q)
                return
            self.send_error(404)

        def do_POST(self):
            if self.path != "/cmd":
                self.send_error(404)
                return
            n = int(self.headers.get("Content-Length", "0"))
            text = self.rfile.read(min(n, 4096)).decode("ascii", "replace").strip()
            for line in text.splitlines():
                if line.strip():
                    sim.commands.put(line.strip())
            self.send_response(204)
            self.end_headers()

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sim", help="path to lbr_sim (default: newest build-dev*/sim/lbr_sim)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=os.path.join(REPO, "sim-out"),
                    help="where the simulated SD logs and USB stream go")
    args = ap.parse_args()

    sim = Sim(find_sim(args.sim), os.path.abspath(args.out), args.seed)
    threading.Thread(target=sim.run, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(sim))
    server.daemon_threads = True
    print("LBR simulator UI: http://127.0.0.1:%d/  (Ctrl+C to stop)" % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
