"""
Steering reaction test. Run with wheels off the ground.

Servo on X_DIR: 0 deg = straight = 1630 us, +deg = LEFT, -deg = RIGHT,
+-18 deg = full lock. Commands each target directly and finishes with a
rapid left/right burst.
"""

import os
import sys
import time

# Pi drivers are in ../pi relative to this test.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))

from stm32_link import STM32Link


def snap(link: STM32Link, deg: float, hold_s: float) -> None:
    """Command `deg` and hold it, re-sending at 25 Hz to feed the 250 ms watchdog."""
    side = "LEFT" if deg > 0 else "RIGHT" if deg < 0 else "centre"
    print(f"   -> {deg:+5.1f} deg  {side}")
    t0 = time.time()
    while time.time() - t0 < hold_s:
        link.set_steering(deg)
        time.sleep(0.04)


def main() -> int:
    print("=" * 60)
    print(" STEERING TEST  -- WHEELS OFF THE GROUND   (fast reaction)")
    print("=" * 60)
    input(" Press Enter when the car is safely lifted... ")

    with STM32Link() as link:
        try:
            # Snap to discrete targets.
            for deg, hold in [(0, 0.6), (+18, 0.6), (-18, 0.6),
                              (+10, 0.5), (-10, 0.5), (0, 0.5)]:
                snap(link, deg, hold)

            # Rapid left/right burst.
            print("   rapid L<->R burst:")
            for _ in range(5):
                snap(link, +18, 0.30)
                snap(link, -18, 0.30)
            snap(link, 0, 0.5)

            print(" done.")
            return 0
        except KeyboardInterrupt:
            print(" interrupted -- stopping.")
            return 1


if __name__ == "__main__":
    sys.exit(main())
