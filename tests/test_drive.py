"""
Manual arrow-key teleop. Car must be on the ground.

Controls (single keypress, no Enter): Up/Down = speed (Down past 0 = reverse, holds
like cruise), Left/Right = steer (auto-centers on release), Space = stop, q/Esc = quit.
Speed capped (default 1.0 m/s, raise with --speed-cap). STM32 250 ms watchdog
neutralises the car if the script dies or USB drops. Steering: +deg = LEFT.
"""

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty

# Pi drivers are in ../pi
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))

from stm32_link import STM32Link

SPEED_STEP = 0.1      # m/s per Up/Down press
STEER_STEP = 6.0      # deg per Left/Right press
STEER_MAX  = 18.0     # deg full lock
STEER_IDLE_S = 0.8    # auto-center steering after this many s with no L/R key
FEED_HZ = 30.0        # re-send rate (feeds the 250 ms watchdog, holds state)


def read_key(fd: int) -> str:
    """Read one keypress from fd, decoding arrows. Returns UP/DOWN/LEFT/RIGHT, a single char, or '' for lone Esc."""
    ch = os.read(fd, 1)
    if ch != b"\x1b":
        return ch.decode("latin1")
    if not select.select([fd], [], [], 0.02)[0]:
        return ""                       # bare Esc, quit
    rest = os.read(fd, 2)               # arrow tail: '[A'..'[D'
    if rest[:1] == b"[":
        return {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}.get(rest[1:2], "?")
    return "?"


def clampf(v, lo, hi):
    return max(lo, min(hi, v))


def main() -> int:
    ap = argparse.ArgumentParser(description="COVAPSY manual arrow-key drive")
    ap.add_argument("--speed-cap", type=float, default=1.0, help="max |speed| m/s (default 1.0)")
    ap.add_argument("--stm32-port", default="/dev/ttyACM0")
    args = ap.parse_args()
    cap = abs(args.speed_cap)

    print("=" * 66)
    print(" MANUAL DRIVE  --  CAR ON THE GROUND, CLEAR SPACE AROUND IT")
    print("   Up/Down = speed (cruise)   Left/Right = steer (auto-centers)")
    print("   Space = STOP        q / Esc = quit")
    print(f"   speed cap: {cap:.1f} m/s   (watchdog stops the car if this exits)")
    print("=" * 66)

    if not sys.stdin.isatty():
        print(" ERROR: needs an interactive terminal (run it directly over SSH, not piped).")
        return 1
    input(" Wheels on the ground, space clear -- press Enter to start... ")

    with STM32Link(args.stm32_port) as link:
        st = {"spd": 0.0, "steer": 0.0, "steer_t": 0.0, "rev_armed": False, "run": True}

        def _hold_speed(mps, dur):
            """Send mps at ~25 Hz for dur seconds (feeds the watchdog)."""
            t0 = time.time()
            while st["run"] and time.time() - t0 < dur:
                link.set_speed(mps)
                time.sleep(0.04)

        # Re-send speed+steer at FEED_HZ (feeds the watchdog, holds cruise) and
        # auto-center the steering after STEER_IDLE_S.
        def feeder():
            period = 1.0 / FEED_HZ
            while st["run"]:
                spd = st["spd"]
                try:
                    # WP-1060 reverse needs a brake, neutral, reverse double-tap to
                    # engage from a stop (a sustained reverse pulse alone just brakes).
                    # Done once on entering reverse. Also requires reverse enabled in
                    # the ESC running mode, else this just brakes twice.
                    if spd < 0 and not st["rev_armed"]:
                        _hold_speed(spd, 0.25)     # tap 1, brake
                        _hold_speed(0.0, 0.12)     # neutral
                        st["rev_armed"] = True
                    elif spd >= 0:
                        st["rev_armed"] = False

                    if time.time() - st["steer_t"] > STEER_IDLE_S:
                        st["steer"] = 0.0
                    link.set_steering(st["steer"])
                    link.set_speed(spd)
                except Exception:
                    return
                side = "LEFT " if st["steer"] > 0 else "RIGHT" if st["steer"] < 0 else "centre"
                tag = "  REV-armed" if st["rev_armed"] else ""
                sys.stdout.write(f"\r SPEED {spd:+.1f} m/s   STEER {st['steer']:+5.1f} deg {side}{tag}"
                                 f"   [Up/Dn  L/R  Space=STOP  q=quit]   ")
                sys.stdout.flush()
                time.sleep(period)

        threading.Thread(target=feeder, daemon=True).start()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                k = read_key(fd)
                if k in ("q", "Q", ""):
                    break
                elif k == "UP":
                    st["spd"] = clampf(st["spd"] + SPEED_STEP, -cap, cap)
                elif k == "DOWN":
                    st["spd"] = clampf(st["spd"] - SPEED_STEP, -cap, cap)
                elif k == "LEFT":
                    st["steer"] = clampf(st["steer"] + STEER_STEP, -STEER_MAX, STEER_MAX)
                    st["steer_t"] = time.time()
                elif k == "RIGHT":
                    st["steer"] = clampf(st["steer"] - STEER_STEP, -STEER_MAX, STEER_MAX)
                    st["steer_t"] = time.time()
                elif k == " ":
                    st["spd"] = 0.0
                    st["steer"] = 0.0
                    st["steer_t"] = time.time()
        finally:
            st["run"] = False
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        time.sleep(0.1)         # let the feeder exit before closing
        link.stop()

    print("\n stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
