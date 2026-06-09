"""
Forward and reverse motor test.

Forward sets pulse to ~1800 us. Reverse uses the STM32 firmware REV
command for ms-accurate brake-release-reverse timing on the Hobbywing
WP-1060-RTR.

Wheels OFF the ground. ESC orientation: BLACK wire on the on/off-switch
side of X_PROP (signal pin is on the far side).
"""

import os
import sys
import time

# Pi drivers are in ../pi
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))

from stm32_link import STM32Link


def _raw(link: STM32Link, line: str) -> None:
    link.send_raw(line)


def _hold_pwm(link: STM32Link, us: int, seconds: float, label: str) -> None:
    print(f" {label}: P{us} for {seconds:.1f}s")
    _raw(link, f"P{us}")
    time.sleep(seconds)


def _drain(link: STM32Link, seconds: float) -> None:
    """Print firmware lines for a fixed window via link.pop_lines (the
    background reader); does not touch link.ser directly."""
    end = time.time() + seconds
    while time.time() < end:
        for line in link.pop_lines():
            print("  STM32: " + line)
        time.sleep(0.05)


def main() -> int:
    print("=" * 60)
    print(" MOTOR TEST  -- WHEELS OFF THE GROUND")
    print(" ESC orientation: BLACK wire toward the on/off switch.")
    print("=" * 60)
    input(" Press Enter to start. ")

    with STM32Link() as link:
        link.set_steering(0.0)
        _raw(link, "WD0")
        time.sleep(0.1)
        try:
            _hold_pwm(link, 1500, 2.0, "neutral (warm-up)")
            _hold_pwm(link, 1800, 3.0, "forward")

            # STM32 firmware drives the brake-release-reverse timing.
            print(" REV  (firmware-driven brake-release-reverse)")
            _raw(link, "REV")
            _drain(link, 8.0)   # spindown 2.5 + brake 0.5 + release 1.2 + reverse 3.0 + neutral 0.5 = 7.7s

            _hold_pwm(link, 1500, 0.5, "neutral")
        finally:
            _raw(link, "WD1")
            time.sleep(0.05)
            link.stop()

    print(" done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
