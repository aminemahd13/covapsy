"""Autonomous race control for the COVAPSY car.

States: IDLE (neutral, waiting for START), RUNNING (LiDAR gap-follow with rear-IR
reverse and stuck recovery), STOPPED (neutral, START re-arms to RUNNING). Boots IDLE.
Start/stop come from a TCP control server, control files in control_dir
(start/stop/quit), or the keyboard (s/x/q) on a TTY. The STM32 250 ms comms
watchdog neutralises the car if this program dies or USB drops.

Usage (Pi, wheels off the ground for first runs):
    python3 race.py                 # autonomous, boots IDLE
    python3 race.py --dry-run       # steering tracks LiDAR, throttle 0
    python3 race.py --camera        # also read RGB wrong-way (advisory only)
"""

import argparse
import os
import queue
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "common"))
import gap_follower as gf          # noqa: E402
from config import PARAMS          # noqa: E402

from lidar import Lidar            # noqa: E402
from stm32_link import STM32Link   # noqa: E402


# --------------------------------------------------------------------------
# Remote start/stop: TCP server, control files, and keyboard feed one
# thread-safe command queue. Commands: START / STOP / QUIT / STATUS.
# --------------------------------------------------------------------------
class Control:
    def __init__(self, port: int, control_dir: str):
        self.port = port
        self.control_dir = control_dir
        try:
            os.makedirs(control_dir, exist_ok=True)
        except OSError:
            pass
        self._events = queue.Queue()
        self._state = "IDLE"
        self._running = True
        threading.Thread(target=self._serve, daemon=True).start()
        threading.Thread(target=self._watch_files, daemon=True).start()
        if sys.stdin and sys.stdin.isatty():
            threading.Thread(target=self._watch_keys, daemon=True).start()

    def set_state(self, s: str) -> None:
        self._state = s

    def poll(self):
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                break
        return out

    def stop(self) -> None:
        self._running = False

    # command sources
    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("0.0.0.0", self.port))
            srv.listen(5)
        except OSError as e:
            print(f"[ctl] cannot bind TCP {self.port}: {e} (file/keyboard still work)")
            return
        srv.settimeout(0.5)
        while self._running:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()
        srv.close()

    def _handle(self, conn):
        with conn:
            conn.settimeout(2.0)
            try:
                cmd = conn.recv(64).decode("ascii", "replace").strip().upper()
            except Exception:
                return
            if cmd in ("START", "STOP", "QUIT"):
                self._events.put(cmd)
                conn.sendall(f"OK {cmd} (state={self._state})\n".encode())
            elif cmd == "STATUS":
                conn.sendall(f"state={self._state}\n".encode())
            else:
                conn.sendall(b"ERR use START/STOP/STATUS/QUIT\n")

    def _watch_files(self):
        while self._running:
            for name, cmd in (("start", "START"), ("stop", "STOP"), ("quit", "QUIT")):
                p = os.path.join(self.control_dir, name)
                if os.path.exists(p):
                    try:
                        os.remove(p)
                    except OSError:
                        pass
                    self._events.put(cmd)
            time.sleep(0.1)

    def _watch_keys(self):
        for line in sys.stdin:
            c = line.strip().lower()
            if c in ("s", "start"):
                self._events.put("START")
            elif c in ("x", "stop"):
                self._events.put("STOP")
            elif c in ("q", "quit"):
                self._events.put("QUIT")


def parse_args():
    ap = argparse.ArgumentParser(description="COVAPSY race control")
    ap.add_argument("--dry-run", action="store_true",
                    help="stream steering only; throttle forced to 0")
    ap.add_argument("--no-steer", action="store_true", help="force steering 0 too")
    ap.add_argument("--camera", action="store_true",
                    help="read the RGB camera for wrong-way (advisory only)")
    ap.add_argument("--speed-cap", type=float, default=None)
    ap.add_argument("--autostart", action="store_true",
                    help="boot straight into RUNNING (TESTING ONLY -- unsafe on the ground)")
    ap.add_argument("--lidar-port", default="/dev/ttyUSB0")
    ap.add_argument("--stm32-port", default="/dev/ttyACM0")
    return ap.parse_args()


def _engage_reverse(link):
    """Brake-then-neutral double-tap that arms WP-1060 reverse before the reverse pulse.

    Without it the ESC only brakes. Raw PWM re-sent at ~25 Hz to feed the 250 ms
    watchdog. Blocks ~0.45 s, only on the transition into reverse.
    """
    for us, dur in (("P1150", 0.30), ("P1500", 0.15)):   # brake (while rolling), then neutral
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            link.send_raw(us)
            time.sleep(0.04)


def main():
    args = parse_args()
    p = dict(PARAMS)
    if args.speed_cap is not None:
        p["max_speed_mps"] = args.speed_cap
    period = 1.0 / p["loop_hz"]
    steer_sign = p["steer_sign"]

    ctl = Control(p["control_port"], p["control_dir"])
    state = "RUNNING" if args.autostart else "IDLE"

    print("=" * 62)
    print(" COVAPSY RACE CONTROL  --  WHEELS OFF THE GROUND for first runs")
    print(f" mode: {'DRY-RUN' if args.dry_run else 'LIVE'}   "
          f"cap {p['max_speed_mps']:.2f} m/s   ctl TCP :{p['control_port']}   "
          f"files {p['control_dir']}/(start|stop|quit)")
    print(" START -> drive,  STOP -> neutral.  (covapsy_ctl.py start|stop)")
    print("=" * 62)

    # Stuck-recovery state.
    stuck_since = None
    recovery_until = 0.0
    recovery_steer = 0.0
    reverse_armed = False   # WP-1060 reverse brake-then-neutral tap already done?

    cam = None
    last_cam = 0.0
    wrong_way = False

    with Lidar(args.lidar_port, front_offset_deg=p["lidar_front_offset_deg"]) as lidar, \
            STM32Link(args.stm32_port) as link:

        if args.camera:
            try:
                from camera import Camera
                cam = Camera().start()
                print(f" camera: {cam.device}")
            except Exception as e:
                print(f" camera unavailable ({e}); continuing without it")

        print(" waiting for LiDAR...")
        t0 = time.monotonic()
        while not lidar.healthy and (time.monotonic() - t0) < 8.0:
            link.stop()
            time.sleep(0.1)
        if not lidar.healthy:
            print(" ERROR: no LiDAR scans. Check /dev/ttyUSB0.")
            ctl.stop()
            return 1
        print(f" LiDAR OK. State = {state}.")

        last_log = 0.0
        try:
            while True:
                tick = time.monotonic()

                for cmd in ctl.poll():
                    if cmd == "START" and state != "RUNNING":
                        state = "RUNNING"
                        stuck_since = None
                        recovery_until = 0.0
                        print(" >> START -> RUNNING")
                    elif cmd == "STOP" and state != "STOPPED":
                        state = "STOPPED"
                        print(" >> STOP -> STOPPED")
                    elif cmd == "QUIT":
                        raise KeyboardInterrupt
                ctl.set_state(state)

                scan = lidar.get_scan()
                rear_blocked = not link.rear_clear()
                front = gf.front_clearance(scan, p)

                if state == "RUNNING" and lidar.healthy:
                    steer_deg, speed_mps = gf.gap_follow(scan, rear_blocked, p)

                    # Stuck: commanding forward but wheel encoder shows no
                    # turning for stuck_secs.
                    if speed_mps > 0.05 and link.wheels_stalled(p["wheel_moving_pps"]):
                        if stuck_since is None:
                            stuck_since = tick
                        elif (tick - stuck_since) > p["stuck_secs"] and tick > recovery_until:
                            # Back out toward the more open side if the rear is clear.
                            lc, rc = gf.side_clearances(scan, p)
                            recovery_steer = p["max_steer_deg"] if lc >= rc else -p["max_steer_deg"]
                            recovery_until = tick + p["recovery_secs"]
                            stuck_since = None
                            print(" !! stuck (wheels not turning) -> recovery reverse")
                    else:
                        stuck_since = None

                    # Active recovery overrides the gap follower.
                    if tick < recovery_until:
                        if not rear_blocked:
                            steer_deg, speed_mps = recovery_steer, -p["reverse_speed_mps"]
                        else:
                            steer_deg, speed_mps = 0.0, 0.0   # boxed both ways, hold
                else:
                    steer_deg, speed_mps = 0.0, 0.0

                if args.no_steer:
                    steer_deg = 0.0
                if args.dry_run:
                    speed_mps = 0.0

                # Send to the car. Reverse arms once via the brake-neutral tap, then
                # drives a raw reverse pulse (the m/s-to-us map is too gentle near
                # neutral). Forward and stop use the normal speed path.
                if speed_mps < 0.0 and not rear_blocked:
                    if not reverse_armed:
                        _engage_reverse(link)
                        reverse_armed = True
                    # Negate steer in reverse: kinematics are mirrored, so turning the
                    # wheels the other way swings the nose toward the open side.
                    link.set_steering(steer_sign * -steer_deg)
                    link.send_raw("P%d" % int(p["reverse_pwm_us"]))
                else:
                    reverse_armed = False
                    link.set_steering(steer_sign * steer_deg)
                    link.set_speed(max(speed_mps, 0.0))

                # Camera wrong-way: advisory telemetry only, not a control input.
                if cam is not None and (tick - last_cam) > 0.2:
                    try:
                        wrong_way = bool(cam.read().get("wrong_way", False))
                    except Exception:
                        wrong_way = False
                    last_cam = tick

                if tick - last_log >= 0.5:
                    head = link.heading_deg
                    act = link.speed_mps or 0.0
                    print(f" [{state:7}] front={front:4.2f}m steer={steer_deg:+6.1f} "
                          f"cmd={speed_mps:+4.2f} act={act:4.2f}m/s "
                          f"rear={'BLK' if rear_blocked else 'clr'} "
                          f"head={head if head is not None else '--'} "
                          f"{'WRONG-WAY' if wrong_way else ''}")
                    last_log = tick

                dt = time.monotonic() - tick
                if dt < period:
                    time.sleep(period - dt)
        except KeyboardInterrupt:
            print("\n quit -> neutralising.")
        finally:
            link.stop()
            ctl.stop()
            if cam is not None:
                cam.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
