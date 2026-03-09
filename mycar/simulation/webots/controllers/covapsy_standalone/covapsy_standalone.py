"""Standalone Webots controller for COVAPSY (no ROS required).

Keyboard controls:
  A: start autonomous mode
  N: stop autonomous mode
  S: switch algorithm (advanced gap follower / simple baseline)
  R: trigger short reverse maneuver
  + / -: increase or decrease speed cap
"""

import math
from controller import Lidar
from vehicle import Driver


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


driver = Driver()
basic_time_step = int(driver.getBasicTimeStep())
sensor_time_step = max(basic_time_step, 32)

lidar = Lidar("RpLidarA2")
lidar.enable(sensor_time_step)
lidar.enablePointCloud()

keyboard = driver.getKeyboard()
keyboard.enable(sensor_time_step)

# Car / sensor constants
MAX_STEERING_DEG = 16.0
MAX_STEERING_RAD = math.radians(MAX_STEERING_DEG)
CAR_WIDTH_M = 0.30
MAX_LIDAR_RANGE_M = 5.0
MIN_VALID_RANGE_M = 0.05
FRONT_FOV_DEG = 100

# Controller tuning
DISPARITY_THRESHOLD_M = 0.35
SAFETY_RADIUS_M = 0.18
MIN_SPEED_M_S = 0.30
DEFAULT_MAX_SPEED_M_S = 1.10
MAX_SPEED_LIMIT_M_S = 5.00
SPEED_RAMP_UP_M_S = 0.015
SPEED_RAMP_DOWN_M_S = 0.12
REVERSE_SPEED_M_S = -0.60
REVERSE_STEPS = 28


def set_drive_command(steering_rad, speed_m_s):
    steering_rad = clamp(steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    # Webots steering sign is inverted versus controller geometry.
    driver.setSteeringAngle(-steering_rad)
    driver.setCruisingSpeed(speed_m_s * 3.6)


def angle_of_index(idx):
    """Convert 0..359 index to signed angle where 0 deg is front."""
    if idx <= 180:
        deg = idx
    else:
        deg = idx - 360
    return math.radians(deg)


def read_lidar_front_zero():
    """Return lidar ranges as 360 values where index 0 points to the front."""
    raw = list(lidar.getRangeImage())
    if not raw:
        return [MAX_LIDAR_RANGE_M] * 360

    count = len(raw)
    out = [MAX_LIDAR_RANGE_M] * 360

    for i in range(360):
        # Match CoVAPSy indexing convention used in professor controllers.
        distance = raw[-i % count]
        mapped_idx = (i - 180) % 360
        if MIN_VALID_RANGE_M <= distance <= MAX_LIDAR_RANGE_M:
            out[mapped_idx] = float(distance)
        else:
            out[mapped_idx] = MAX_LIDAR_RANGE_M

    return out


def front_sector(ranges):
    indices = [i for i in range(360) if abs(math.degrees(angle_of_index(i))) <= FRONT_FOV_DEG]
    return indices, [ranges[i] for i in indices], [angle_of_index(i) for i in indices]


def find_largest_gap(mask):
    """Return (start, end) for largest contiguous True run."""
    best = None
    start = None

    for i, valid in enumerate(mask):
        if valid and start is None:
            start = i
        elif not valid and start is not None:
            candidate = (start, i - 1)
            if best is None or (candidate[1] - candidate[0]) > (best[1] - best[0]):
                best = candidate
            start = None

    if start is not None:
        candidate = (start, len(mask) - 1)
        if best is None or (candidate[1] - candidate[0]) > (best[1] - best[0]):
            best = candidate

    return best


def advanced_gap_follower(ranges, speed_cap_m_s):
    front_indices, front_ranges, front_angles = front_sector(ranges)
    if not front_ranges:
        return 0.0, 0.0

    # 1) Disparity extension
    half_width = CAR_WIDTH_M * 0.5
    angle_step = (2.0 * math.pi) / 360.0
    for i in range(1, len(front_ranges)):
        if abs(front_ranges[i] - front_ranges[i - 1]) > DISPARITY_THRESHOLD_M:
            closer_idx = i if front_ranges[i] < front_ranges[i - 1] else i - 1
            closer_range = front_ranges[closer_idx]
            if closer_range > MIN_VALID_RANGE_M:
                extend_angle = math.atan2(half_width, closer_range)
                extend_count = int(extend_angle / angle_step)
                lo = max(0, closer_idx - extend_count)
                hi = min(len(front_ranges), closer_idx + extend_count + 1)
                for j in range(lo, hi):
                    front_ranges[j] = min(front_ranges[j], closer_range)

    # 2) Safety bubble around nearest obstacle
    nearest_idx = min(range(len(front_ranges)), key=front_ranges.__getitem__)
    nearest_dist = front_ranges[nearest_idx]
    if nearest_dist < MAX_LIDAR_RANGE_M:
        nearest_angle = front_angles[nearest_idx]
        for i in range(len(front_ranges)):
            arc_dist = nearest_dist * abs(front_angles[i] - nearest_angle)
            if arc_dist < SAFETY_RADIUS_M:
                front_ranges[i] = 0.0

    # 3) Largest free gap
    traversable = [r > MIN_VALID_RANGE_M for r in front_ranges]
    best_gap = find_largest_gap(traversable)
    if best_gap is None:
        return 0.0, 0.0

    # 4) Best point = deepest point in largest gap
    gap_start, gap_end = best_gap
    best_local = max(
        range(gap_start, gap_end + 1),
        key=lambda idx: front_ranges[idx],
    )
    target_angle = front_angles[best_local]

    # 5) Steering + speed adaptation
    steering_rad = clamp(target_angle, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    steering_factor = 1.0 - 0.80 * abs(steering_rad) / MAX_STEERING_RAD
    clearance_factor = clamp(nearest_dist / 2.0, 0.25, 1.0)
    speed_m_s = MIN_SPEED_M_S + (speed_cap_m_s - MIN_SPEED_M_S) * steering_factor * clearance_factor
    speed_m_s = clamp(speed_m_s, MIN_SPEED_M_S, speed_cap_m_s)
    return steering_rad, speed_m_s


def simple_baseline(ranges, speed_cap_m_s):
    # Legacy baseline based on left/right range difference (CoVAPSy teaching baseline).
    left_mm = ranges[60] * 1000.0
    right_mm = ranges[300] * 1000.0
    steering_deg = 0.02 * (left_mm - right_mm)
    steering_rad = math.radians(clamp(steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG))
    return steering_rad, min(0.60, speed_cap_m_s)


def emergency_front_blocked(ranges):
    front_window = list(range(348, 360)) + list(range(0, 13))
    nearest = min(ranges[idx] for idx in front_window)
    return nearest < 0.14


auto_mode = False
use_advanced = True
reverse_ticks = 0
stuck_counter = 0
speed_cap = DEFAULT_MAX_SPEED_M_S
smoothed_speed = 0.0

print("=" * 64)
print("COVAPSY Standalone Webots Controller")
print("No ROS required. Click the 3D window then use:")
print("  A=start  N=stop  S=algo toggle  R=reverse  +/- speed cap")
print("=" * 64)

set_drive_command(0.0, 0.0)

while driver.step() != -1:
    while True:
        key = keyboard.getKey()
        if key == -1:
            break
        if key in (ord("A"), ord("a")):
            auto_mode = True
            reverse_ticks = 0
            stuck_counter = 0
            print("[standalone] auto mode ON")
        elif key in (ord("N"), ord("n")):
            auto_mode = False
            reverse_ticks = 0
            smoothed_speed = 0.0
            set_drive_command(0.0, 0.0)
            print("[standalone] auto mode OFF")
        elif key in (ord("S"), ord("s")):
            use_advanced = not use_advanced
            mode = "ADVANCED_GAP_FOLLOWER" if use_advanced else "SIMPLE_BASELINE"
            print(f"[standalone] algorithm={mode}")
        elif key in (ord("R"), ord("r")):
            reverse_ticks = REVERSE_STEPS
            print("[standalone] reverse maneuver")
        elif key in (ord("+"), ord("=")):
            speed_cap = min(MAX_SPEED_LIMIT_M_S, speed_cap + 0.1)
            print(f"[standalone] speed_cap={speed_cap:.2f} m/s")
        elif key in (ord("-"), ord("_")):
            speed_cap = max(MIN_SPEED_M_S, speed_cap - 0.1)
            print(f"[standalone] speed_cap={speed_cap:.2f} m/s")

    if not auto_mode:
        continue

    ranges = read_lidar_front_zero()

    if reverse_ticks > 0:
        reverse_ticks -= 1
        set_drive_command(0.0, REVERSE_SPEED_M_S)
        continue

    if emergency_front_blocked(ranges):
        stuck_counter += 1
        if stuck_counter > 8:
            reverse_ticks = REVERSE_STEPS
            stuck_counter = 0
            print("[standalone] emergency reverse (front blocked)")
        set_drive_command(0.0, 0.0)
        continue

    stuck_counter = max(0, stuck_counter - 1)

    if use_advanced:
        steering, target_speed = advanced_gap_follower(ranges, speed_cap)
    else:
        steering, target_speed = simple_baseline(ranges, speed_cap)

    if target_speed > smoothed_speed:
        smoothed_speed = min(target_speed, smoothed_speed + SPEED_RAMP_UP_M_S)
    else:
        smoothed_speed = max(target_speed, smoothed_speed - SPEED_RAMP_DOWN_M_S)

    set_drive_command(steering, smoothed_speed)
