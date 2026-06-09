"""
Find the steering center pulse for a freshly attached servo.

Arrow keys drive the raw DIR<us> pulse live (no Enter) over the full 1000..2000 us
travel. Press 'c' to print the pulse to set as DIR_MILIEU in stm32/Core/Inc/moteurs.h.
Keep wheels off the ground; the pulse is re-sent in the background to hold it.

Keys: left/right move by the current step, up/down change step,
c marks center, 0 jumps to 1630 us, q or Esc quits.
"""

import os
import select
import sys
import termios
import threading
import time
import tty

# Pi drivers are in ../pi relative to this tests dir.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))

from stm32_link import STM32Link

# Must match stm32/Core/Inc/moteurs.h and the DIR_CAL_* clamp in main.c.
# This servo: straight = 1630 us, left = lower us, right = higher us, +deg = LEFT.
DIR_MILIEU = 1630
DIR_BUTEE_GAUCHE = 1370                      # left endpoint (lower us)
DIR_BUTEE_DROITE = 1890                      # right endpoint (higher us)
US_PER_DEG = (DIR_BUTEE_DROITE - DIR_BUTEE_GAUCHE) / (2 * 18.0)   # 14.44 us/deg
US_MIN, US_MAX = 1000, 2000                 # firmware DIR_CAL clamp (full travel)

STEPS = [1, 2, 5, 10, 25, 50]               # us per arrow press


def nominal_deg(us: int) -> float:
    # +deg = LEFT (lower us).
    return (DIR_MILIEU - us) / US_PER_DEG


def clamp(us: int) -> int:
    return max(US_MIN, min(US_MAX, us))


def read_key(fd: int) -> str:
    """Block for one keypress, decoding arrow escape sequences via raw os.read.

    Returns LEFT/RIGHT/UP/DOWN, a single char, or '' for a lone Esc.
    """
    ch = os.read(fd, 1)
    if ch != b"\x1b":
        return ch.decode("latin1")
    if not select.select([fd], [], [], 0.02)[0]:
        return ""                       # bare Esc, quit
    rest = os.read(fd, 2)               # arrow tail: '[A'..'[D'
    if rest[:1] == b"[":
        return {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}.get(rest[1:2], "?")
    return "?"


def status(us: int, step: int) -> None:
    sys.stdout.write(f"\r pulse {us} us  (angle {nominal_deg(us):+5.1f} deg)  "
                     f"step {step} us       ")
    sys.stdout.flush()


def main() -> int:
    print("=" * 66)
    print(" STEERING CENTER CALIBRATION  --  WHEELS OFF THE GROUND")
    print(" Arrow keys move the servo live (no Enter), full travel:")
    print("   <- more left   -> more right   ^ bigger step   v smaller step")
    print("   c = mark center    0 = 1630us (centre)    q / Esc = quit")
    print("=" * 66)

    if not sys.stdin.isatty():
        print(" ERROR: needs an interactive terminal (run it directly over SSH, not piped).")
        return 1

    with STM32Link() as link:
        state = {"us": DIR_MILIEU, "run": True}

        # Re-send the current pulse at 20 Hz to hold it past the 250 ms watchdog.
        def feeder():
            while state["run"]:
                try:
                    link.send_raw(f"DIR{state['us']}")
                except Exception:
                    return
                time.sleep(0.05)

        threading.Thread(target=feeder, daemon=True).start()

        step_i = 3                                  # 10 us
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            status(state["us"], STEPS[step_i])
            while True:
                k = read_key(fd)
                if k in ("q", "Q", ""):
                    break
                elif k == "LEFT":
                    state["us"] = clamp(state["us"] - STEPS[step_i])   # left = lower us
                elif k == "RIGHT":
                    state["us"] = clamp(state["us"] + STEPS[step_i])   # right = higher us
                elif k == "UP":
                    step_i = min(len(STEPS) - 1, step_i + 1)
                elif k == "DOWN":
                    step_i = max(0, step_i - 1)
                elif k == "0":
                    state["us"] = DIR_MILIEU
                elif k in ("c", "C"):
                    us = state["us"]
                    sys.stdout.write(
                        f"\r *** CENTER pulse {us} us  ->  set DIR_MILIEU = {us} in moteurs.h "
                        f"(was {DIR_MILIEU}, {us-DIR_MILIEU:+d} us, ~{nominal_deg(us):+.1f} deg) ***\n")
                    sys.stdout.flush()
                status(state["us"], STEPS[step_i])
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        state["run"] = False
        time.sleep(0.12)                            # let feeder exit before close
        link.stop()

    print("\n done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
