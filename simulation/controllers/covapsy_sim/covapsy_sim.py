# Copyright 2026 COVAPSY Team
#
# Webots controller for the TT-02 car. Runs the shared follow-the-gap code
# (common/gap_follower.py): read LiDAR, call gap_follow(), drive.
#
# Usage:
#   1. Set the TT02 robot's "controller" field to "covapsy_sim".
#   2. Click the 3D view, press 'A' to drive autonomously, 'N' to stop.

import os
import sys
from math import radians
from pathlib import Path

from vehicle import Driver
from controller import Lidar

# --- Import the shared code ------------------------------------------------
# Shared follow-the-gap code lives in common/. Walk up the tree to find it so
# the import works regardless of where Webots places the controller.
_COMMON = None
for _p in Path(__file__).resolve().parents:
    if (_p / "common" / "gap_follower.py").is_file():
        _COMMON = _p / "common"
        break
if _COMMON is None:
    raise RuntimeError("covapsy_sim: could not locate common/ (shared code)")
sys.path.insert(0, str(_COMMON))
import gap_follower  # noqa: E402
import config  # noqa: E402

PARAMS = config.PARAMS

# Optional file logging. Set COVAPSY_LOGFILE=<path> to enable (Webots GUI does
# not pipe stdout to a terminal on Windows); harmless when unset.
_LOGF = None
_logpath = os.environ.get("COVAPSY_LOGFILE")
if _logpath:
    _LOGF = open(_logpath, "w", buffering=1)


def _log(msg):
    print(msg)
    if _LOGF:
        _LOGF.write(msg + "\n")
        _LOGF.flush()


_log(f"[COVAPSY] controller import OK; brain={_COMMON}")

# --- Webots setup ----------------------------------------------------------

driver = Driver()
basic_step = int(driver.getBasicTimeStep())
sensor_step = 4 * basic_step

lidar = Lidar("RpLidarA2")
lidar.enable(sensor_step)

keyboard = driver.getKeyboard()
keyboard.enable(sensor_step)


def read_scan():
    """Webots LiDAR to canonical scan: 360 floats in metres, index 0 = front,
    increasing index = counter-clockwise (index 90 = left, 270 = right).
    No-return or invalid beams are 0.0 (downstream treats that as max range).
    """
    raw = lidar.getRangeImage()
    scan = [0.0] * 360
    if raw is None:
        return scan
    n = len(raw)
    for i in range(360):
        d = raw[-i % n]
        # Webots returns inf for no-return; clamp inf and <=0 to 0.0.
        if d == d and 0.0 < d < float("inf"):  # d == d rejects NaN
            scan[(i - 180) % 360] = d
        else:
            scan[(i - 180) % 360] = 0.0
    return scan


def rear_nearest(scan, ignore_self=True):
    """Nearest real obstacle behind the car, within +/- rear_sector_deg of
    index 180. By default ignore returns nearer than rear_self_ignore_m (the
    TT-02 shell occludes the rear LiDAR with a constant close return)."""
    sec = int(PARAMS["rear_sector_deg"])
    mx = PARAMS["max_range_m"]
    self_r = PARAMS["rear_self_ignore_m"]
    nearest = mx
    for d in range(-sec, sec + 1):
        r = scan[(180 + d) % 360]
        if r <= 0.0:                       # no return, treat as clear
            continue
        if ignore_self and r < self_r:     # own body, ignore
            continue
        if r < nearest:
            nearest = r
    return nearest


def rear_is_blocked(scan):
    """True if a real obstacle is within rear_block_m behind the car."""
    return rear_nearest(scan) < PARAMS["rear_block_m"]


# --- Main loop -------------------------------------------------------------

print("=" * 52)
print("  COVAPSY sim controller -- shared gap_follower brain")
print("  Click the 3D view, then:")
print("    'A' = enable autonomous driving")
print("    'N' = stop (zero speed + steering, disable auto)")
print("=" * 52)

# COVAPSY_AUTOSTART=1 enables auto mode at boot; normally press 'A' in the
# 3D view.
auto_mode = os.environ.get("COVAPSY_AUTOSTART") == "1"
if auto_mode:
    print("[COVAPSY] autostart (COVAPSY_AUTOSTART=1)")
driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

step_count = 0
while driver.step() != -1:
    step_count += 1
    # Keyboard
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        if key in (ord("a"), ord("A")):
            if not auto_mode:
                auto_mode = True
                print("[COVAPSY] Auto mode ENABLED")
        elif key in (ord("n"), ord("N")):
            if auto_mode:
                auto_mode = False
                print("[COVAPSY] Auto mode DISABLED")

    if not auto_mode:
        driver.setSteeringAngle(0.0)
        driver.setCruisingSpeed(0.0)
        continue

    scan = read_scan()
    rear_blocked = rear_is_blocked(scan)
    steer_deg, speed_mps = gap_follower.gap_follow(scan, rear_blocked)

    # Webots steering sign is inverted vs our convention (+left, -right). In
    # reverse, mirror the steer so the nose swings toward the open side.
    steer_cmd = steer_deg if speed_mps < 0 else -steer_deg
    driver.setSteeringAngle(radians(steer_cmd))
    driver.setCruisingSpeed(speed_mps * 3.6)  # m/s to km/h (negative = reverse)

    # Heartbeat (~1 Hz) so headless runs show the car is driving.
    if step_count % 30 == 0:
        front = gap_follower.front_clearance(scan)
        _log(f"[COVAPSY] step={step_count} front={front:4.2f}m "
             f"steer={steer_deg:+6.1f} speed={speed_mps:+4.2f} "
             f"rear_raw={rear_nearest(scan, ignore_self=False):4.2f} "
             f"rear_obj={rear_nearest(scan):4.2f} rear_blk={rear_blocked}")
