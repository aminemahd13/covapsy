# Copyright 2026 COVAPSY Team
#
# Advanced autonomous controller for TT-02 car in CoVAPSy Webots simulator
# Implements: Follow-the-Gap with Disparity Extension + speed adaptation
# Compatible with: Webots R2025a, Piste_CoVAPSy_2025a.wbt
#
# Usage:
#   1. Copy this folder into my-simulation/Simulation_CoVAPSy_Webots/controllers/
#   2. In Webots, set the TT02 car's controller to "covapsy_controller"
#   3. Press 'A' in 3D view to enable auto mode

from vehicle import Driver
from controller import Lidar
import numpy as np
import math
import time

# ─── Initialize Webots ───────────────────────────────────────────────

driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# LiDAR
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# Keyboard
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# ─── Car parameters (matching TT-02 1/10 scale) ─────────────────────

MAX_SPEED_KMH = 28.0
MAX_ANGLE_DEG = 16.0
CAR_WIDTH = 0.20          # meters (1/10 scale TT-02)
WHEELBASE = 0.257         # meters

# ─── Navigation parameters ──────────────────────────────────────────

# Gap follower
SAFETY_RADIUS = 0.15      # meters - bubble around nearest obstacle
DISPARITY_THRESHOLD = 0.3 # meters - triggers disparity extension
MAX_RANGE = 5.0           # meters - clip lidar beyond this

# Speed control
MAX_SPEED_MS = 2.0        # starting conservative
MIN_SPEED_MS = 0.3
SPEED_RAMP_STEP = 0.01    # m/s per step for smooth acceleration

# Steering
STEERING_GAIN = 1.2       # proportional gain for angle-to-steering

# ─── State variables ────────────────────────────────────────────────

auto_mode = False
current_speed_target = 0.0
smooth_speed = 0.0
lap_count = 0

# ─── Helper functions ───────────────────────────────────────────────

def set_speed_m_s(speed_m_s):
    """Set speed in m/s, converted to km/h for Webots driver API."""
    speed_kmh = speed_m_s * 3.6
    speed_kmh = max(0.0, min(speed_kmh, MAX_SPEED_KMH))
    driver.setCruisingSpeed(speed_kmh)

def set_steering_degrees(angle_degrees):
    """Set steering in degrees, converted to radians (negated for Webots coords)."""
    angle_degrees = max(-MAX_ANGLE_DEG, min(MAX_ANGLE_DEG, angle_degrees))
    angle_rad = -angle_degrees * math.pi / 180.0
    driver.setSteeringAngle(angle_rad)

def reverse():
    """Emergency reverse maneuver."""
    driver.setCruisingSpeed(-1.0)

def read_lidar():
    """Read and convert Webots lidar data to 360-element mm array.
    Index 0 = front, positive angles = left (trigonometric convention),
    matching the professor's STM32 convention."""
    raw = lidar.getRangeImage()
    if raw is None:
        return np.zeros(360)

    lidar_mm = np.zeros(360)
    for i in range(360):
        distance = raw[-i % len(raw)]
        if 0.0 < distance < 20.0:
            lidar_mm[(i - 180) % 360] = int(1000 * distance)
        else:
            lidar_mm[(i - 180) % 360] = 0
    return lidar_mm


# ─── Follow-the-Gap with Disparity Extension ────────────────────────

def gap_follower(lidar_mm):
    """Advanced gap-following algorithm.

    Steps:
      1. Convert lidar_mm to float array, clip to max range
      2. Apply disparity extension (inflate obstacles at depth discontinuities)
      3. Create safety bubble around nearest obstacle
      4. Find the largest gap
      5. Steer toward the deepest point in the largest gap
      6. Adapt speed to steering angle and nearest obstacle distance
    """
    # Work with meters
    ranges = lidar_mm.astype(np.float64) / 1000.0
    n = len(ranges)

    # Replace zeros with a small value (zero = no reading)
    ranges[ranges < 0.01] = 0.01

    # Clip to max range
    ranges = np.clip(ranges, 0.01, MAX_RANGE)

    # Angles: 0=front, positive=left, negative=right (trigonometric)
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)

    # Only consider front 180 degrees (-90 to +90) for gap finding
    front_mask = np.abs(angles) <= math.radians(100)
    front_indices = np.where(front_mask)[0]

    if len(front_indices) == 0:
        return 0.0, 0.0

    front_ranges = ranges[front_indices].copy()
    front_angles = angles[front_indices]

    # ── Step 1: Disparity extension ──
    half_width = CAR_WIDTH / 2.0
    angle_inc = 2.0 * math.pi / n

    for i in range(1, len(front_ranges)):
        diff = abs(front_ranges[i] - front_ranges[i - 1])
        if diff > DISPARITY_THRESHOLD:
            closer_idx = i if front_ranges[i] < front_ranges[i - 1] else i - 1
            closer_range = front_ranges[closer_idx]
            if closer_range > 0.05:
                extend_angle = math.atan2(half_width, closer_range)
                extend_count = int(extend_angle / angle_inc)
                lo = max(0, closer_idx - extend_count)
                hi = min(len(front_ranges), closer_idx + extend_count + 1)
                for j in range(lo, hi):
                    front_ranges[j] = min(front_ranges[j], closer_range)

    # ── Step 2: Safety bubble around nearest obstacle ──
    nearest_idx = np.argmin(front_ranges)
    nearest_dist = front_ranges[nearest_idx]

    if nearest_dist < SAFETY_RADIUS * 3:
        for i in range(len(front_ranges)):
            angular_dist = abs(front_angles[i] - front_angles[nearest_idx])
            arc_dist = nearest_dist * angular_dist
            if arc_dist < SAFETY_RADIUS:
                front_ranges[i] = 0.0

    # ── Step 3: Find largest gap ──
    nonzero = front_ranges > 0.05
    gaps = []
    start = None

    for i in range(len(front_ranges)):
        if nonzero[i] and start is None:
            start = i
        elif not nonzero[i] and start is not None:
            gaps.append((start, i - 1))
            start = None
    if start is not None:
        gaps.append((start, len(front_ranges) - 1))

    if not gaps:
        return 0.0, 0.0  # Emergency stop

    # ── Step 4: Choose best point in largest gap ──
    best_gap = max(gaps, key=lambda g: g[1] - g[0])
    gap_ranges = front_ranges[best_gap[0]:best_gap[1] + 1]
    best_in_gap = np.argmax(gap_ranges)
    best_idx = best_gap[0] + best_in_gap
    target_angle = front_angles[best_idx]

    # Convert to degrees for steering
    steering_deg = STEERING_GAIN * math.degrees(target_angle)
    steering_deg = max(-MAX_ANGLE_DEG, min(MAX_ANGLE_DEG, steering_deg))

    # ── Step 5: Speed adaptation ──
    # Slow down for tight steering and close obstacles
    steer_factor = 1.0 - 0.7 * abs(steering_deg) / MAX_ANGLE_DEG
    dist_factor = min(1.0, nearest_dist / 1.0)  # Slow if obstacle < 1m
    speed = MIN_SPEED_MS + (MAX_SPEED_MS - MIN_SPEED_MS) * steer_factor * dist_factor
    speed = max(MIN_SPEED_MS, min(MAX_SPEED_MS, speed))

    return steering_deg, speed


# ─── Simple reactive controller (professor's baseline) ──────────────

def simple_reactive(lidar_mm):
    """Professor's simple reactive algorithm for comparison.
    angle = 0.02 * (lidar[60] - lidar[300])"""
    angle_deg = 0.02 * (lidar_mm[60] - lidar_mm[300])
    return angle_deg, 0.5


# ─── Emergency detection ────────────────────────────────────────────

def check_emergency(lidar_mm):
    """Check if car is stuck or about to crash."""
    front_30 = lidar_mm[345:360].tolist() + lidar_mm[0:15].tolist()
    min_front = min(front_30) if front_30 else 999

    if min_front < 100:  # Less than 10cm in front
        return True
    return False


# ─── Control mode selection ─────────────────────────────────────────

USE_ADVANCED = True  # Set to False to use professor's simple algorithm

# ─── Main loop ──────────────────────────────────────────────────────

print("=" * 50)
print("  COVAPSY Autonomous Racing Controller")
print("  Click on the 3D view, then press 'A' to start")
print("  Press 'N' to stop, 'R' for reverse")
print("  Press 'S' to switch algorithm (simple/advanced)")
print("=" * 50)

set_steering_degrees(0)
set_speed_m_s(0)

stuck_counter = 0

while driver.step() != -1:
    # ── Keyboard handling ──
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        elif key == ord('n') or key == ord('N'):
            if auto_mode:
                auto_mode = False
                set_speed_m_s(0)
                set_steering_degrees(0)
                smooth_speed = 0.0
                print("[COVAPSY] Auto mode DISABLED")
        elif key == ord('a') or key == ord('A'):
            if not auto_mode:
                auto_mode = True
                stuck_counter = 0
                print("[COVAPSY] Auto mode ENABLED")
        elif key == ord('r') or key == ord('R'):
            print("[COVAPSY] Reversing...")
            reverse()
        elif key == ord('s') or key == ord('S'):
            USE_ADVANCED = not USE_ADVANCED
            algo_name = "ADVANCED (gap-follower)" if USE_ADVANCED else "SIMPLE (professor baseline)"
            print(f"[COVAPSY] Switched to: {algo_name}")

    # ── Read LiDAR ──
    lidar_mm = read_lidar()

    if not auto_mode:
        set_steering_degrees(0)
        set_speed_m_s(0)
        continue

    # ── Emergency check ──
    if check_emergency(lidar_mm):
        stuck_counter += 1
        if stuck_counter > 10:
            print("[COVAPSY] STUCK - reversing!")
            reverse()
            stuck_counter = 0
            continue
    else:
        stuck_counter = max(0, stuck_counter - 1)

    # ── Run selected algorithm ──
    if USE_ADVANCED:
        steering_deg, speed_target = gap_follower(lidar_mm)
    else:
        steering_deg, speed_target = simple_reactive(lidar_mm)

    # ── Smooth speed transitions ──
    if speed_target > smooth_speed:
        smooth_speed = min(speed_target, smooth_speed + SPEED_RAMP_STEP)
    else:
        smooth_speed = max(speed_target, smooth_speed - SPEED_RAMP_STEP * 3)

    # ── Apply commands ──
    set_steering_degrees(steering_deg)
    set_speed_m_s(smooth_speed)
