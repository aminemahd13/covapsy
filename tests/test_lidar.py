"""
RPLIDAR check, run on the Pi.

Runs ~5 s at ~10 Hz; each tick prints beam count, front distance,
nearest beam, and left/right distances.

Index 0 = front, increasing index = CCW = left (90 = 90 deg left,
270 = 90 deg right). 0.0 = no return.
"""

import os
import sys
import time

# gap_follower in ../common, Pi drivers in ../pi
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "common"))
sys.path.insert(0, os.path.join(_HERE, "..", "pi"))

import gap_follower
from lidar import Lidar


def _nearest(scan):
    """(distance_m, angle_deg) of the closest returning beam, else (0.0, -1)."""
    best_d = None
    best_a = -1
    for a, d in enumerate(scan):
        if d > 0.0 and (best_d is None or d < best_d):
            best_d = d
            best_a = a
    return (best_d if best_d is not None else 0.0), best_a


def main() -> int:
    print("=" * 60)
    print(" LIDAR TEST -- ~5 s at ~10 Hz")
    print("=" * 60)

    saw_data = False
    try:
        with Lidar() as lidar:
            end = time.time() + 5.0
            while time.time() < end:
                scan = lidar.get_scan()
                nonzero = sum(1 for d in scan if d > 0.0)
                front = gap_follower.front_clearance(scan)
                near_d, near_a = _nearest(scan)
                left = scan[90]
                right = scan[270]
                if nonzero:
                    saw_data = True
                print(
                    f" beams={nonzero:3d}  front={front:5.2f}m  "
                    f"near={near_d:5.2f}m@{near_a:3d}deg  "
                    f"L(90)={left:5.2f}m  R(270)={right:5.2f}m  "
                    f"healthy={lidar.healthy}"
                )
                time.sleep(0.1)
    except KeyboardInterrupt:
        print(" interrupted -- stopping.")
        return 1

    if saw_data:
        print(" OK -- lidar returned data, scan looks sane.")
        return 0
    print(" FAIL -- no beams ever returned; check wiring / port / baud.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
