"""Standalone Webots controller for COVAPSY (no ROS required).

Keyboard controls:
  A: start autonomous mode
  N: stop autonomous mode
  S: switch algorithm (advanced gap follower / simple baseline)
  R: trigger short reverse maneuver
  + / -: increase or decrease speed cap
"""

import math

try:
    from controller import Lidar
    from vehicle import Driver

    WEBOTS_AVAILABLE = True
except Exception:
    Lidar = object
    Driver = object
    WEBOTS_AVAILABLE = False


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


# Car / sensor constants
MAX_STEERING_DEG = 16.0
MAX_STEERING_RAD = math.radians(MAX_STEERING_DEG)
CAR_WIDTH_M = 0.30
MAX_LIDAR_RANGE_M = 5.0
MIN_VALID_RANGE_M = 0.05
FRONT_FOV_DEG = 100

# Controller tuning
DISPARITY_THRESHOLD_M = 0.30
SAFETY_RADIUS_M = 0.22
MIN_SPEED_FLOOR_M_S = 0.10
BASE_MIN_SPEED_M_S = 0.24
RACE_PROFILE_SPEED_CAPS = {
    "HOMOLOGATION": 0.8,
    "RACE_STABLE": 2.5,
    "RACE_AGGRESSIVE": 3.2,
}
DEFAULT_RACE_PROFILE = "RACE_STABLE"
DEFAULT_MAX_SPEED_M_S = RACE_PROFILE_SPEED_CAPS[DEFAULT_RACE_PROFILE]
MAX_SPEED_LIMIT_M_S = RACE_PROFILE_SPEED_CAPS["RACE_AGGRESSIVE"]
SPEED_RAMP_UP_M_S = 0.030
SPEED_RAMP_DOWN_M_S = 0.10
REVERSE_SPEED_M_S = -0.55
REVERSE_STEPS = 24
EMERGENCY_STUCK_STEPS = 8
EMERGENCY_FRONT_WINDOW_DEG = 12
EMERGENCY_BLOCK_DISTANCE_M = 0.14

# Stability knobs
ANGLE_PENALTY_PER_RAD = 0.35
WRONG_SIDE_DAMPING = 0.35
CLEARANCE_DIFF_THRESHOLD_M = 0.18
SIDE_CLEAR_MIN_DEG = 22
SIDE_CLEAR_MAX_DEG = 85
STEERING_RATE_LIMIT_RAD = math.radians(1.25)
STEERING_LOW_PASS_ALPHA = 0.55
REVERSE_STEERING_MAX_RAD = math.radians(10.0)


def set_drive_command(driver, steering_rad, speed_m_s):
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


def read_lidar_front_zero(lidar):
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
    """Return front sector in monotonic angle order: -FOV ... +FOV."""
    angle_degrees = range(-FRONT_FOV_DEG, FRONT_FOV_DEG + 1)
    indices = [deg % 360 for deg in angle_degrees]
    front_ranges = [float(ranges[idx]) for idx in indices]
    front_angles = [math.radians(deg) for deg in angle_degrees]
    return indices, front_ranges, front_angles


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


def median_or_default(values, default):
    if not values:
        return default
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def side_clearances(front_ranges, front_angles):
    min_rad = math.radians(SIDE_CLEAR_MIN_DEG)
    max_rad = math.radians(SIDE_CLEAR_MAX_DEG)

    left = [r for r, a in zip(front_ranges, front_angles) if min_rad <= a <= max_rad]
    right = [r for r, a in zip(front_ranges, front_angles) if -max_rad <= a <= -min_rad]

    left_clear = median_or_default(left, MAX_LIDAR_RANGE_M)
    right_clear = median_or_default(right, MAX_LIDAR_RANGE_M)
    return left_clear, right_clear


def apply_turn_direction_sanity(steering_rad, front_ranges, front_angles):
    left_clear, right_clear = side_clearances(front_ranges, front_angles)

    # If steering points into the tighter side, damp it toward center.
    if steering_rad > 0.0 and (left_clear + CLEARANCE_DIFF_THRESHOLD_M) < right_clear:
        return steering_rad * WRONG_SIDE_DAMPING
    if steering_rad < 0.0 and (right_clear + CLEARANCE_DIFF_THRESHOLD_M) < left_clear:
        return steering_rad * WRONG_SIDE_DAMPING
    return steering_rad


def adaptive_min_speed(nearest_dist, steering_rad):
    distance_factor = clamp(nearest_dist / 1.0, 0.0, 1.0)
    turn_factor = 1.0 - 0.6 * clamp(abs(steering_rad) / MAX_STEERING_RAD, 0.0, 1.0)
    min_speed = MIN_SPEED_FLOOR_M_S + (BASE_MIN_SPEED_M_S - MIN_SPEED_FLOOR_M_S) * distance_factor * turn_factor
    return clamp(min_speed, MIN_SPEED_FLOOR_M_S, BASE_MIN_SPEED_M_S)


def advanced_gap_follower(ranges, speed_cap_m_s):
    _front_indices, front_ranges, front_angles = front_sector(ranges)
    if not front_ranges:
        return 0.0, 0.0

    reference_ranges = front_ranges.copy()

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
        arc_scale = max(nearest_dist, MIN_VALID_RANGE_M)
        for i in range(len(front_ranges)):
            arc_dist = arc_scale * abs(front_angles[i] - nearest_angle)
            if arc_dist < SAFETY_RADIUS_M:
                front_ranges[i] = 0.0

    # 3) Largest free gap
    traversable = [r > MIN_VALID_RANGE_M for r in front_ranges]
    best_gap = find_largest_gap(traversable)
    if best_gap is None:
        return 0.0, 0.0

    # 4) Best point in largest gap with center bias.
    gap_start, gap_end = best_gap
    best_local = max(
        range(gap_start, gap_end + 1),
        key=lambda idx: front_ranges[idx] - ANGLE_PENALTY_PER_RAD * abs(front_angles[idx]),
    )
    target_angle = front_angles[best_local]

    # 5) Steering + speed adaptation
    steering_rad = clamp(target_angle, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    steering_rad = apply_turn_direction_sanity(steering_rad, reference_ranges, front_angles)

    steering_factor = 1.0 - 0.80 * abs(steering_rad) / MAX_STEERING_RAD
    clearance_factor = clamp((nearest_dist - 0.12) / 1.6, 0.0, 1.0)

    adaptive_floor = adaptive_min_speed(nearest_dist, steering_rad)
    cap = max(adaptive_floor, speed_cap_m_s)
    speed_m_s = adaptive_floor + (cap - adaptive_floor) * steering_factor * clearance_factor
    speed_m_s = clamp(speed_m_s, adaptive_floor, cap)
    return steering_rad, speed_m_s


def simple_baseline(ranges, speed_cap_m_s):
    # Legacy baseline based on left/right range difference (CoVAPSy teaching baseline).
    left_mm = ranges[60] * 1000.0
    right_mm = ranges[300] * 1000.0
    steering_deg = 0.02 * (left_mm - right_mm)
    steering_rad = math.radians(clamp(steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG))
    return steering_rad, min(0.55, speed_cap_m_s)


def emergency_front_blocked(ranges):
    front_window = [deg % 360 for deg in range(-EMERGENCY_FRONT_WINDOW_DEG, EMERGENCY_FRONT_WINDOW_DEG + 1)]
    nearest = min(ranges[idx] for idx in front_window)
    return nearest < EMERGENCY_BLOCK_DISTANCE_M


def nearest_front_obstacle(ranges, half_window_deg=45):
    nearest_dist = MAX_LIDAR_RANGE_M
    nearest_angle = 0.0
    for deg in range(-half_window_deg, half_window_deg + 1):
        dist = ranges[deg % 360]
        if dist < nearest_dist:
            nearest_dist = dist
            nearest_angle = math.radians(deg)
    return nearest_dist, nearest_angle


def compute_reverse_steering(ranges):
    nearest_dist, nearest_angle = nearest_front_obstacle(ranges, half_window_deg=45)
    if nearest_dist >= (MAX_LIDAR_RANGE_M * 0.95):
        return 0.0

    severity = clamp((0.60 - nearest_dist) / 0.60, 0.25, 1.0)
    steer_mag = REVERSE_STEERING_MAX_RAD * severity

    if abs(nearest_angle) < math.radians(8.0):
        _idx, front_ranges, front_angles = front_sector(ranges)
        left_clear, right_clear = side_clearances(front_ranges, front_angles)
        direction = 1.0 if left_clear > right_clear else -1.0
    else:
        # Reverse steer away from nearest front obstacle side.
        direction = -1.0 if nearest_angle > 0.0 else 1.0

    steering_rad = direction * steer_mag
    return clamp(steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)


def stabilize_steering_command(target_steering_rad, prev_steering_rad):
    target_steering_rad = clamp(target_steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    delta = target_steering_rad - prev_steering_rad
    limited_delta = clamp(delta, -STEERING_RATE_LIMIT_RAD, STEERING_RATE_LIMIT_RAD)
    rate_limited = prev_steering_rad + limited_delta

    smoothed = (
        (1.0 - STEERING_LOW_PASS_ALPHA) * prev_steering_rad
        + STEERING_LOW_PASS_ALPHA * rate_limited
    )
    return clamp(smoothed, -MAX_STEERING_RAD, MAX_STEERING_RAD)


def run_controller():
    if not WEBOTS_AVAILABLE:
        raise RuntimeError("Webots Python API is not available in this environment.")

    driver = Driver()
    basic_time_step = int(driver.getBasicTimeStep())
    sensor_time_step = max(basic_time_step, 32)

    lidar = Lidar("RpLidarA2")
    lidar.enable(sensor_time_step)
    lidar.enablePointCloud()

    keyboard = driver.getKeyboard()
    keyboard.enable(sensor_time_step)

    auto_mode = False
    use_advanced = True
    reverse_ticks = 0
    reverse_steering = 0.0
    stuck_counter = 0
    speed_cap = DEFAULT_MAX_SPEED_M_S
    smoothed_speed = 0.0
    prev_steering = 0.0
    last_ranges = [MAX_LIDAR_RANGE_M] * 360

    print("=" * 64)
    print("COVAPSY Standalone Webots Controller")
    print("No ROS required. Click the 3D window then use:")
    print("  A=start  N=stop  S=algo toggle  R=reverse  +/- speed cap")
    print(f"Active speed profile: {DEFAULT_RACE_PROFILE} (cap={speed_cap:.2f} m/s)")
    print("=" * 64)

    set_drive_command(driver, 0.0, 0.0)

    while driver.step() != -1:
        while True:
            key = keyboard.getKey()
            if key == -1:
                break
            if key in (ord("A"), ord("a")):
                auto_mode = True
                reverse_ticks = 0
                reverse_steering = 0.0
                stuck_counter = 0
                smoothed_speed = 0.0
                prev_steering = 0.0
                print("[standalone] auto mode ON")
            elif key in (ord("N"), ord("n")):
                auto_mode = False
                reverse_ticks = 0
                reverse_steering = 0.0
                smoothed_speed = 0.0
                prev_steering = 0.0
                set_drive_command(driver, 0.0, 0.0)
                print("[standalone] auto mode OFF")
            elif key in (ord("S"), ord("s")):
                use_advanced = not use_advanced
                mode = "ADVANCED_GAP_FOLLOWER" if use_advanced else "SIMPLE_BASELINE"
                print(f"[standalone] algorithm={mode}")
            elif key in (ord("R"), ord("r")):
                reverse_ticks = REVERSE_STEPS
                reverse_steering = compute_reverse_steering(last_ranges)
                print(
                    "[standalone] reverse maneuver "
                    f"(steer={math.degrees(reverse_steering):.1f} deg)"
                )
            elif key in (ord("+"), ord("=")):
                speed_cap = min(MAX_SPEED_LIMIT_M_S, speed_cap + 0.1)
                print(f"[standalone] speed_cap={speed_cap:.2f} m/s")
            elif key in (ord("-"), ord("_")):
                speed_cap = max(MIN_SPEED_FLOOR_M_S, speed_cap - 0.1)
                print(f"[standalone] speed_cap={speed_cap:.2f} m/s")

        if not auto_mode:
            continue

        ranges = read_lidar_front_zero(lidar)
        last_ranges = ranges

        if reverse_ticks > 0:
            reverse_ticks -= 1
            prev_steering = reverse_steering
            set_drive_command(driver, reverse_steering, REVERSE_SPEED_M_S)
            continue

        if emergency_front_blocked(ranges):
            stuck_counter += 1
            if stuck_counter > EMERGENCY_STUCK_STEPS:
                reverse_ticks = REVERSE_STEPS
                reverse_steering = compute_reverse_steering(ranges)
                stuck_counter = 0
                print(
                    "[standalone] emergency reverse (front blocked) "
                    f"steer={math.degrees(reverse_steering):.1f} deg"
                )
            set_drive_command(driver, 0.0, 0.0)
            continue

        stuck_counter = max(0, stuck_counter - 1)

        if use_advanced:
            steering, target_speed = advanced_gap_follower(ranges, speed_cap)
        else:
            steering, target_speed = simple_baseline(ranges, speed_cap)

        steering = stabilize_steering_command(steering, prev_steering)
        prev_steering = steering

        if target_speed > smoothed_speed:
            smoothed_speed = min(target_speed, smoothed_speed + SPEED_RAMP_UP_M_S)
        else:
            smoothed_speed = max(target_speed, smoothed_speed - SPEED_RAMP_DOWN_M_S)

        set_drive_command(driver, steering, smoothed_speed)


if __name__ == "__main__":
    run_controller()
