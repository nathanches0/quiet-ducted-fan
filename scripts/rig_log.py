#!/usr/bin/env python3
"""
Bench-rig logger for the Quiet Ducted Fan.

Talks to firmware/rig_logger on the Arduino Nano, sends the safety heartbeat,
and saves every run to results/ as a CSV with the design written at the top.

Interactive (type the firmware's commands, e.g. a, t 40, f 8, s):
    python scripts/rig_log.py --port COM5 --label 11-9-23 --gap 1.5

Equal-thrust sweep (the test plan's method: same thrust for every design):
    python scripts/rig_log.py --port COM5 --label 15-7-23 --sweep 4,8,12

Try it now without any hardware:
    python scripts/rig_log.py --sim --label test --sweep 4,8

List serial ports:
    python scripts/rig_log.py --list

Needs: pip install pyserial
"""
import argparse
import csv
import datetime as dt
import math
import os
import queue
import random
import threading
import time

HEADER = "ms,armed,upper_pct,lower_pct,thrust_N,rpm_upper,rpm_lower,current_A,voltage_V,power_W,hold_N"
FIELDS = HEADER.split(",")
HEARTBEAT_S = 0.5


# --------------------------------------------------------------------------- simulated rig
class SimRig:
    """Stands in for the Arduino so the logger can be tried without hardware.
    Same commands and CSV as the firmware; the fan follows the design model
    (16.3 N and 316 W at 100 %), so the numbers are illustrative only."""

    def __init__(self):
        self.lines = queue.Queue()
        self.t0 = time.time()
        self.armed = False
        self.tgt = [0.0, 0.0]
        self.out = [0.0, 0.0]
        self.rpm = [0.0, 0.0]
        self.hold = -1.0
        self.base = 0.0
        self.trim = 0.0
        self.last_rx = time.time()
        self.alive = True
        self.lines.put("# Quiet Ducted Fan rig ready (SIMULATED). Disarmed. Send ? for help.")
        self.lines.put(HEADER)
        threading.Thread(target=self._run, daemon=True).start()

    def _msg(self, m):
        self.lines.put("# " + m)

    def _stop(self, why):
        self.armed = False
        self.tgt = [0.0, 0.0]
        self.out = [0.0, 0.0]
        self.hold = -1
        self._msg("STOP: " + why)

    def write(self, data):
        self.last_rx = time.time()
        for raw in data.decode().splitlines():
            cmd = raw.strip().lower()
            if not cmd:
                continue
            c, arg = cmd[0], cmd[1:].strip()
            try:
                v = float(arg) if arg else None
            except ValueError:
                v = None
            if c == "p":
                continue
            if c == "s":
                self._stop("command")
            elif c == "a":
                if any(self.tgt):
                    self._msg("set throttle to 0 before arming")
                else:
                    self.armed = True
                    self._msg("ARMED")
            elif c in "tul":
                if not self.armed:
                    self._msg("not armed")
                elif v is not None:
                    self.hold = -1
                    v = max(0.0, min(100.0, v))
                    if c in "tu":
                        self.tgt[0] = v
                    if c in "tl":
                        self.tgt[1] = max(0.0, min(100.0, v + (self.trim if c == "t" else 0)))
            elif c == "k" and v is not None:
                self.trim = max(-20.0, min(20.0, v))
            elif c == "f":
                if not self.armed:
                    self._msg("not armed")
                elif v and v > 0:
                    self.hold, self.base = v, self.tgt[0]
                    self._msg("thrust hold on")
                else:
                    self.hold = -1
                    self._msg("thrust hold off")
            elif c == "?":
                self._msg(f"armed={int(self.armed)} (simulated rig)")
            else:
                self._msg("command accepted (simulated)")

    def _run(self):
        last = time.time()
        while self.alive:
            time.sleep(0.05)
            now = time.time()
            step = now - last
            last = now
            if self.armed and now - self.last_rx > 1.5:
                self._stop("watchdog (laptop went quiet)")
            thrust = 16.3 * sum((r / 15300) ** 2 for r in self.rpm) / 2
            if self.armed and self.hold > 0:
                self.base = max(0.0, min(100.0, self.base + 3.0 * (self.hold - thrust) * step))
                self.tgt = [self.base, max(0.0, min(100.0, self.base + self.trim))]
            for i in range(2):
                self.out[i] += max(-20 * step, min(20 * step, self.tgt[i] - self.out[i]))
                self.rpm[i] += (16000 * self.out[i] / 100 - self.rpm[i]) * min(1.0, step / 0.15)
            power = 316 * sum((r / 15300) ** 3 for r in self.rpm) / 2
            volts = 16.0 - 0.02 * power / 16.0
            amps = power / volts
            ms = int((now - self.t0) * 1000)
            row = [ms, int(self.armed), f"{self.out[0]:.1f}", f"{self.out[1]:.1f}",
                   f"{thrust + random.gauss(0, 0.02):.3f}",
                   f"{max(0.0, self.rpm[0] + random.gauss(0, 30)):.0f}", f"{max(0.0, self.rpm[1] + random.gauss(0, 30)):.0f}",
                   f"{amps:.2f}", f"{volts:.2f}", f"{amps * volts:.1f}",
                   f"{self.hold if self.hold > 0 else 0:.2f}"]
            self.lines.put(",".join(str(x) for x in row))

    def readline(self):
        try:
            return (self.lines.get(timeout=0.3) + "\n").encode()
        except queue.Empty:
            return b""

    def close(self):
        self.alive = False


# --------------------------------------------------------------------------- logger
class Logger:
    def __init__(self, link, path, meta, echo_every):
        self.link = link
        self.path = path
        self.lock = threading.Lock()
        self.latest = None
        self.phase = "manual"
        self.point = ""
        self.rows = []          # (phase, point, row dict) kept for the sweep summary
        self.running = True
        self.echo_every = echo_every
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.f = open(path, "w", newline="")
        for k, v in meta.items():
            self.f.write(f"# {k}: {v}\n")
        self.w = csv.writer(self.f)
        self.w.writerow(FIELDS + ["phase", "point_N"])
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        if echo_every > 0:
            threading.Thread(target=self._echo, daemon=True).start()

    def send(self, cmd):
        with self.lock:
            self.link.write((cmd.strip() + "\n").encode())

    def _heartbeat(self):
        while self.running:
            try:
                self.send("p")
            except Exception:
                pass
            time.sleep(HEARTBEAT_S)

    def _read(self):
        while self.running:
            try:
                raw = self.link.readline()
            except Exception as e:
                print(f"\n! serial error: {e}")
                break
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)
                continue
            if line.startswith("ms,"):
                continue
            parts = line.split(",")
            if len(parts) != len(FIELDS):
                continue
            try:
                row = {k: float(v) for k, v in zip(FIELDS, parts)}
            except ValueError:
                continue
            self.latest = row
            self.w.writerow(parts + [self.phase, self.point])
            self.rows.append((self.phase, self.point, row))

    def _echo(self):
        while self.running:
            time.sleep(self.echo_every)
            print("  " + status(self.latest))

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.f.close()


def status(r):
    if not r:
        return "(no data yet)"
    eff = r["thrust_N"] / r["power_W"] * 1000 if r["power_W"] > 1 else float("nan")
    return (f"{'ARMED' if r['armed'] else 'safe '} | thr {r['upper_pct']:5.1f}/{r['lower_pct']:5.1f} % | "
            f"{r['thrust_N']:6.2f} N | {r['rpm_upper']:6.0f}/{r['rpm_lower']:6.0f} rpm | "
            f"{r['current_A']:5.1f} A {r['voltage_V']:5.2f} V {r['power_W']:6.1f} W | "
            f"{eff:5.1f} N/kW")


def summarise(log, out_path):
    """Average each recorded sweep point and print/save a summary table."""
    pts = {}
    for phase, point, r in log.rows:
        if phase == "record":
            pts.setdefault(point, []).append(r)
    if not pts:
        return
    keys = ["thrust_N", "power_W", "rpm_upper", "rpm_lower", "current_A", "voltage_V", "upper_pct", "lower_pct"]
    print("\nSweep summary (averages over each recording window)")
    print(f"{'target N':>8} {'thrust N':>9} {'±':>6} {'power W':>8} {'N/kW':>6} {'rpm up':>7} {'rpm lo':>7} {'thr %':>6}")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["target_N", "n_rows"] + [k + "_mean" for k in keys] + ["thrust_N_sd", "N_per_kW"])
        for point, rs in pts.items():
            m = {k: sum(r[k] for r in rs) / len(rs) for k in keys}
            sd = math.sqrt(sum((r["thrust_N"] - m["thrust_N"]) ** 2 for r in rs) / max(1, len(rs) - 1))
            eff = m["thrust_N"] / m["power_W"] * 1000 if m["power_W"] > 1 else float("nan")
            w.writerow([point, len(rs)] + [f"{m[k]:.3f}" for k in keys] + [f"{sd:.3f}", f"{eff:.1f}"])
            print(f"{point:>8} {m['thrust_N']:9.2f} {sd:6.2f} {m['power_W']:8.1f} {eff:6.1f} "
                  f"{m['rpm_upper']:7.0f} {m['rpm_lower']:7.0f} {m['upper_pct']:6.1f}")
    print(f"Summary saved to {out_path}")


def wait_until(pred, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.05)
    return False


def main():
    ap = argparse.ArgumentParser(description="Quiet Ducted Fan bench-rig logger")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/tty.usbserial-1410")
    ap.add_argument("--sim", action="store_true", help="use a simulated rig (no hardware)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--label", default="run", help="design under test, e.g. 11-9-23")
    ap.add_argument("--gap", default="", help="rotor-stator gap in chords, e.g. 1.5")
    ap.add_argument("--notes", default="", help="anything else worth recording")
    ap.add_argument("--out", default="results", help="folder for CSV files")
    ap.add_argument("--sweep", help="thrust points in newtons, e.g. 4,8,12 (runs automatically)")
    ap.add_argument("--settle", type=float, default=4.0, help="minimum seconds to settle at each point")
    ap.add_argument("--max-settle", type=float, default=40.0, help="give up settling after this many seconds")
    ap.add_argument("--record", type=float, default=10.0, help="seconds to record at each point")
    ap.add_argument("--echo", type=float, default=2.0, help="seconds between status lines (0 = off)")
    args = ap.parse_args()

    if args.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:25} {p.description}")
        return

    if args.sim:
        link = SimRig()
        source = "simulated"
    else:
        if not args.port:
            ap.error("give --port (see --list), or --sim to try it without hardware")
        import serial
        link = serial.Serial(args.port, 115200, timeout=0.3)
        source = args.port
        print("Connecting (the Nano restarts when the port opens)...")
        time.sleep(2.0)

    stamp = dt.datetime.now()
    safe_label = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in args.label)
    path = os.path.join(args.out, f"{stamp:%Y-%m-%d_%H%M%S}_{safe_label}.csv")
    meta = {"design": args.label, "gap_chords": args.gap, "notes": args.notes,
            "source": source, "started": stamp.isoformat(timespec="seconds"),
            "mic": "record separately with the UMIK-1; note the file name here"}
    log = Logger(link, path, meta, args.echo)
    print(f"Logging to {path}")

    try:
        if args.sweep:
            points = [float(x) for x in args.sweep.split(",") if x.strip()]
            if not wait_until(lambda: log.latest is not None, 8):
                print("! no data from the rig: check the port and firmware")
                return
            if not args.sim:
                print("Waiting 3 s for the ESCs to finish starting...")
                time.sleep(3)
            print("Sweep: make sure the arming plug is IN and everyone is behind the shield.")
            log.send("a")
            if not wait_until(lambda: log.latest and log.latest["armed"] == 1, 3):
                print("! rig did not arm (see messages above)")
                return
            for p in points:
                log.phase, log.point = "settle", f"{p:g}"
                print(f"\n-> holding {p:g} N, waiting until it's steady")
                log.send(f"f {p:g}")
                t0, steady_since = time.time(), None
                while time.time() - t0 < args.max_settle:
                    r = log.latest
                    if not r or r["armed"] != 1:
                        print("! rig disarmed during the sweep (see messages above)")
                        return
                    if abs(r["thrust_N"] - p) <= max(0.1, 0.03 * p):
                        steady_since = steady_since or time.time()
                    else:
                        steady_since = None
                    if steady_since and time.time() - steady_since >= 1.5 and time.time() - t0 >= args.settle:
                        break
                    time.sleep(0.05)
                else:
                    print(f"! couldn't hold {p:g} N steadily (max throttle reached?): recording anyway")
                log.phase = "record"
                print(f"   recording {args.record:g} s (start the mic now if it isn't running)")
                time.sleep(args.record)
            log.phase, log.point = "manual", ""
            log.send("s")
            time.sleep(0.5)
            summarise(log, path.replace(".csv", "_summary.csv"))
        else:
            print("Type commands (a arm, t 40, f 8, s stop, ? help). q to quit.")
            while True:
                cmd = input()
                if cmd.strip().lower() in ("q", "quit", "exit"):
                    break
                if cmd.strip():
                    log.send(cmd)
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        try:
            log.send("s")
            time.sleep(0.3)
        except Exception:
            pass
        log.close()
        link.close()
        print(f"Stopped. Data saved to {path}")


if __name__ == "__main__":
    main()
