"""Standalone Webots controller for COVAPSY (no ROS required).

Keyboard controls:
  A: start autonomous mode
  N: stop autonomous mode
  S: switch algorithm (advanced gap follower / simple baseline)
  R: trigger short reverse maneuver
  + / -: increase or decrease speed cap
  L: toggle runtime debug logs
"""

import math
import os
import sys

# Allow importing shared AI modules from the ROS2 workspace (pure-Python, no ROS2 dep)
_RACING_INTEL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                 "ros2_ws", "src", "covapsy_nav", "covapsy_nav")
)
if os.path.isdir(_RACING_INTEL_DIR) and _RACING_INTEL_DIR not in sys.path:
    sys.path.insert(0, _RACING_INTEL_DIR)

try:
    from racing_intelligence import (
        extract_sector_ranges,
        estimate_curvature,
        compute_optimal_speed,
        compute_optimal_speed_fused,
        fuse_curvature_with_imu,
    )
    AI_SPEED_AVAILABLE = True
except ImportError:
    AI_SPEED_AVAILABLE = False

# Import the EKF vehicle state estimator for IMU fusion
_STATE_ESTIMATOR_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                 "ros2_ws", "src", "covapsy_nav", "covapsy_nav")
)
if os.path.isdir(_STATE_ESTIMATOR_DIR) and _STATE_ESTIMATOR_DIR not in sys.path:
    sys.path.insert(0, _STATE_ESTIMATOR_DIR)

try:
    from vehicle_state_estimator import VehicleStateEstimator
    STATE_ESTIMATOR_AVAILABLE = True
except ImportError:
    STATE_ESTIMATOR_AVAILABLE = False

try:
    from controller import Camera, Lidar, RangeFinder, InertialUnit, GPS
    from vehicle import Driver

    WEBOTS_AVAILABLE = True
except Exception:
    Camera = object
    Lidar = object
    RangeFinder = object
    InertialUnit = object
    GPS = object
    Driver = object
    WEBOTS_AVAILABLE = False


# AI speed state (shared across calls for ramp smoothing)
_ai_prev_speed = 0.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def env_flag(name, default=False):
    raw = os.getenv(name)
    if raw is None:
        return default
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def env_float(name, default):
    raw = os.getenv(name)
    if raw is None:
        return float(default)
    try:
        return float(raw)
    except Exception:
        return float(default)


def env_text(name, default):
    raw = os.getenv(name)
    if raw is None:
        return str(default)
    value = str(raw).strip()
    if not value:
        return str(default)
    return value


def ackermann_safe_center_limit(inner_limit_rad, wheelbase_m, track_front_m, margin_rad):
    """Return max center steering so Ackermann inner wheel remains below inner_limit_rad."""
    tan_inner = math.tan(float(inner_limit_rad))
    if abs(tan_inner) < 1e-9:
        return 0.0
    center_limit = math.atan(
        float(wheelbase_m) / ((float(wheelbase_m) / tan_inner) + (0.5 * float(track_front_m)))
    )
    return max(0.0, center_limit - float(margin_rad))


# Car / sensor constants
WEBOTS_WHEELBASE_M = 0.257
WEBOTS_TRACK_FRONT_M = 0.15
WEBOTS_STEERING_INNER_LIMIT_RAD = 0.35
WEBOTS_STEERING_MARGIN_RAD = 5e-4
MAX_STEERING_RAD = ackermann_safe_center_limit(
    inner_limit_rad=WEBOTS_STEERING_INNER_LIMIT_RAD,
    wheelbase_m=WEBOTS_WHEELBASE_M,
    track_front_m=WEBOTS_TRACK_FRONT_M,
    margin_rad=WEBOTS_STEERING_MARGIN_RAD,
)
MAX_STEERING_DEG = math.degrees(MAX_STEERING_RAD)
CAR_WIDTH_M = 0.30
MAX_LIDAR_RANGE_M = 5.0
MIN_VALID_RANGE_M = 0.05
FRONT_FOV_DEG = 150
GAP_SELECTION_FOV_DEG = 90

# Controller tuning
DISPARITY_THRESHOLD_M = 0.38
SAFETY_RADIUS_M = 0.18
MIN_SPEED_FLOOR_M_S = 0.18
BASE_MIN_SPEED_M_S = 0.36
RACE_PROFILE_SPEED_CAPS = {
    "HOMOLOGATION": 0.8,
    "RACE_STABLE": 2.5,
    "RACE_AGGRESSIVE": 2.8,
}
DEFAULT_RACE_PROFILE = "RACE_AGGRESSIVE"
DEFAULT_MAX_SPEED_M_S = RACE_PROFILE_SPEED_CAPS[DEFAULT_RACE_PROFILE]
MAX_SPEED_LIMIT_M_S = RACE_PROFILE_SPEED_CAPS["RACE_AGGRESSIVE"]
SPEED_RAMP_UP_M_S = 0.22
SPEED_RAMP_DOWN_M_S = 0.18
REVERSE_SPEED_M_S = -0.60
REVERSE_STEPS = 28
EMERGENCY_STUCK_STEPS = 6
EMERGENCY_FRONT_WINDOW_DEG = 12
EMERGENCY_BLOCK_DISTANCE_M = 0.17
EMERGENCY_MIN_BLOCK_HITS = 3
LIDAR_MEDIAN_FILTER_RADIUS = 2

# Stability knobs
ANGLE_PENALTY_PER_RAD = 0.12
WRONG_SIDE_DAMPING = 0.55
CLEARANCE_DIFF_THRESHOLD_M = 0.22
SIDE_CLEAR_MIN_DEG = 22
SIDE_CLEAR_MAX_DEG = 85
STEERING_RATE_LIMIT_RAD = math.radians(7.5)
STEERING_LOW_PASS_ALPHA = 0.88
REVERSE_STEERING_MAX_RAD = math.radians(13.0)
GAP_SCORE_WIDTH_WEIGHT = 0.35
GAP_SCORE_CLEARANCE_WEIGHT = 0.35
GAP_SCORE_HEADING_WEIGHT = 0.30
GAP_EDGE_PENALTY = 0.14
SPEED_LOOKAHEAD_WINDOW_DEG = 20
SPEED_TTC_WINDOW_DEG = 12
SPEED_TARGET_TTC_SEC = 0.80
FUSION_CLEARANCE_WEIGHT = 0.45
FUSION_TTC_WEIGHT = 0.35
FUSION_STABILITY_WEIGHT = 0.20

# Forward-direction safety
EMERGENCY_WALL_DIST_M = 0.30
FORWARD_TTC_MIN_SEC = 0.40
SLEW_URGENCY_DIST_M = 0.55
SLEW_URGENCY_BOOST = 2.5

# Bubble fallback when primary safety mask removes all traversable points.
FALLBACK_BUBBLE_RADIUS_SCALE = 0.45
FALLBACK_BUBBLE_MIN_RADIUS_M = 0.05
FALLBACK_BUBBLE_MAX_SPAN_DEG = 55.0

# Cornering and anti-stall knobs
CORNER_TRIGGER_DIST_M = 0.70
CORNER_BIAS_MAX_RAD = math.radians(5.0)
CORNER_CLEARANCE_REF_M = 1.0
NO_PROGRESS_SPEED_CMD_M_S = 0.28
NO_PROGRESS_SPEED_ACTUAL_M_S = 0.05
NO_PROGRESS_FRONT_DIST_M = 0.50
NO_PROGRESS_TRIGGER_STEPS = 15
EARLY_STALL_WINDOW_DIST_M = 0.30
EARLY_STALL_TRIGGER_STEPS = 10

# Curvature-aware speed control
CURVATURE_LOOKAHEAD_HALF_DEG = 40
CURVATURE_SPEED_REDUCTION_FACTOR = 0.25
CURVATURE_TRIGGER_RATIO = 0.55

# Escalating recovery
RECOVERY_ESCALATION_WINDOW_STEPS = 300  # ~9.6 seconds at 32ms
REVERSE_STEPS_LONG = 45
REVERSE_SPEED_HARD_M_S = -0.70
POST_RECOVERY_OVERRIDE_STEPS = 8  # ticks to force counter-steer after recovery

# Camera guidance knobs (color + depth)
RGB_CAMERA_CANDIDATES = ("RealSenseRGB", "RGB_camera", "camera")
DEPTH_CAMERA_CANDIDATES = ("RealSenseDepth", "Depth_camera", "range-finder")
EXPECTED_RED_ON_LEFT = True
CAMERA_ROI_START_FRAC = 0.58
CAMERA_SAMPLE_STRIDE_X = 3
CAMERA_SAMPLE_STRIDE_Y = 3
CAMERA_MIN_BRIGHTNESS = 36
CAMERA_MIN_SATURATION = 0.22
CAMERA_RED_DOMINANCE = 1.30
CAMERA_GREEN_DOMINANCE = 1.25
CAMERA_MIN_COLOR_PIXELS = 80
CAMERA_CONFIDENCE_PIXELS = 850
CAMERA_BLEND_MIN = 0.18
CAMERA_BLEND_MAX = 0.55
CAMERA_LANE_GAIN_RAD = math.radians(12.0)
CAMERA_SINGLE_BORDER_STEER_RAD = math.radians(8.0)
CAMERA_DEPTH_GAIN_RAD = math.radians(5.5)
CAMERA_DEPTH_BALANCE_RANGE_M = 0.40
CAMERA_DEPTH_MIN_VALID_M = 0.12
CAMERA_DEPTH_MAX_VALID_M = 6.0
CAMERA_ORDER_MIN_SEPARATION_PX = 24
CAMERA_WRONG_ORDER_SPEED_CAP_M_S = 1.10
CAMERA_WRONG_ORDER_CONFIDENCE = 0.72
CAMERA_MIN_DEPTH_SAMPLES = 18
CAMERA_DEPTH_PERCENTILE = 0.20
CAMERA_SINGLE_BORDER_CONFIDENCE_CAP = 0.45
CAMERA_SINGLE_BORDER_BLEND_SCALE = 0.55

# U-turn (wrong-direction recovery)
UTURN_TRIGGER_FRAMES = 15         # consecutive wrong-order frames to trigger (~0.5 s)
UTURN_CONFIDENCE_THRESHOLD = 0.65 # min order_confidence to count frame
UTURN_BRAKE_STEPS = 8             # phase 1: brake to stop
UTURN_REVERSE_STEPS = 35          # phase 2: reverse with max steering
UTURN_FORWARD_STEPS = 25          # phase 3: forward with max steering to complete turn
UTURN_REVERSE_SPEED_M_S = -0.65   # reverse speed during U-turn
UTURN_FORWARD_SPEED_M_S = 0.60    # forward speed during turn completion
UTURN_STEER_RAD = MAX_STEERING_RAD  # full lock steering during U-turn

# Sensor-fusion env toggles
CAMERA_GUIDANCE_ENV = "COVAPSY_USE_CAMERA_GUIDANCE"
DEPTH_GUIDANCE_ENV = "COVAPSY_USE_DEPTH_GUIDANCE"
USE_CLOSE_FAR_FUSION_ENV = "COVAPSY_USE_CLOSE_FAR_FUSION"
REACTIVE_FAR_CENTER_GAIN_ENV = "COVAPSY_REACTIVE_FAR_CENTER_GAIN"
REACTIVE_CAMERA_CENTER_GAIN_ENV = "COVAPSY_REACTIVE_CAMERA_CENTER_GAIN"
REACTIVE_FAR_WEIGHT_MIN_ENV = "COVAPSY_REACTIVE_FAR_WEIGHT_MIN"
REACTIVE_FAR_WEIGHT_MAX_ENV = "COVAPSY_REACTIVE_FAR_WEIGHT_MAX"
REACTIVE_FUSION_CLEARANCE_REF_ENV = "COVAPSY_REACTIVE_FUSION_CLEARANCE_REF_M"

TRAFFIC_MODE_ENV = "COVAPSY_TRAFFIC_MODE"
OPPONENT_DETECT_RANGE_ENV = "COVAPSY_OPPONENT_DETECT_RANGE_M"
FOLLOW_DISTANCE_ENV = "COVAPSY_FOLLOW_DISTANCE_M"
TACTICAL_NEAR_WEIGHT_BASE_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_BASE"
TACTICAL_NEAR_WEIGHT_MIN_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_MIN"
TACTICAL_NEAR_WEIGHT_MAX_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_MAX"
TACTICAL_NEAR_WEIGHT_CLEAR_REF_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_CLEAR_REF_DIST"
TACTICAL_NEAR_WEIGHT_TRAFFIC_BOOST_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_TRAFFIC_BOOST"
TACTICAL_NEAR_WEIGHT_CLEARANCE_BOOST_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_CLEARANCE_BOOST"
TACTICAL_NEAR_WEIGHT_STEER_DISAGREEMENT_BOOST_ENV = "COVAPSY_TACTICAL_NEAR_WEIGHT_STEER_DISAGREEMENT_BOOST"
PASS_LOCK_SEC_ENV = "COVAPSY_PASS_LOCK_SEC"

DEFAULT_USE_CLOSE_FAR_FUSION = True
DEFAULT_REACTIVE_FAR_CENTER_GAIN = 0.35
DEFAULT_REACTIVE_CAMERA_CENTER_GAIN = 0.25
DEFAULT_REACTIVE_FAR_WEIGHT_MIN = 0.10
DEFAULT_REACTIVE_FAR_WEIGHT_MAX = 0.55
DEFAULT_REACTIVE_FUSION_CLEARANCE_REF_M = 1.8

DEFAULT_TRAFFIC_MODE = "balanced"
DEFAULT_OPPONENT_DETECT_RANGE_M = 2.8
DEFAULT_FOLLOW_DISTANCE_M = 1.1
DEFAULT_PASS_LOCK_SEC = 0.65
DEFAULT_STEERING_BIAS_GAIN = 0.18
DEFAULT_TACTICAL_TARGET_TTC_SEC = 1.35
DEFAULT_NEAR_WEIGHT_BASE = 0.35
DEFAULT_NEAR_WEIGHT_MIN = 0.20
DEFAULT_NEAR_WEIGHT_MAX = 0.90
DEFAULT_NEAR_WEIGHT_CLEAR_REF_DIST = 1.8
DEFAULT_NEAR_WEIGHT_TRAFFIC_BOOST = 0.35
DEFAULT_NEAR_WEIGHT_CLEARANCE_BOOST = 0.30
DEFAULT_NEAR_WEIGHT_STEER_DISAGREEMENT_BOOST = 0.20
OPPONENT_CONFIDENCE_MIN = 0.30

# Runtime logging controls
STANDALONE_LOG_EVERY_STEPS = 8
STANDALONE_LOG_ENV = "COVAPSY_STANDALONE_LOGS"


def set_drive_command(driver, steering_rad, speed_m_s):
    steering_rad = clamp(steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    # Webots steering sign is inverted versus controller geometry.
    driver.setSteeringAngle(-steering_rad)
    driver.setCruisingSpeed(speed_m_s * 3.6)


def try_enable_optional_sensor(driver, candidate_names, sensor_time_step):
    for device_name in candidate_names:
        try:
            device = driver.getDevice(device_name)
        except Exception:
            continue
        if device is None:
            continue
        try:
            device.enable(sensor_time_step)
            return device, device_name
        except Exception:
            continue
    return None, None


def valid_depth_value(depth_value):
    return (
        depth_value is not None
        and math.isfinite(depth_value)
        and CAMERA_DEPTH_MIN_VALID_M <= depth_value <= CAMERA_DEPTH_MAX_VALID_M
    )


def angle_of_index(idx):
    """Convert 0..359 index to signed angle where 0 deg is front."""
    if idx <= 180:
        deg = idx
    else:
        deg = idx - 360
    return math.radians(deg)


def _quantile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    pos = clamp(int(q * (len(ordered) - 1)), 0, len(ordered) - 1)
    return float(ordered[pos])


def _median_filter_circular(values, radius):
    if radius <= 0:
        return list(values)
    n = len(values)
    if n == 0:
        return []
    window = (2 * radius) + 1
    out = [MAX_LIDAR_RANGE_M] * n
    for i in range(n):
        sample = [values[(i + k) % n] for k in range(-radius, radius + 1)]
        out[i] = median_or_default(sample, values[i]) if len(sample) == window else values[i]
    return out


def read_lidar_front_zero(lidar):
    """Return lidar ranges as 360 values where index 0 points to the front."""
    raw = list(lidar.getRangeImage())
    if not raw:
        return [MAX_LIDAR_RANGE_M] * 360

    count = len(raw)
    if count <= 0:
        return [MAX_LIDAR_RANGE_M] * 360

    sanitized = []
    for distance in raw:
        if MIN_VALID_RANGE_M <= distance <= MAX_LIDAR_RANGE_M and math.isfinite(distance):
            sanitized.append(float(distance))
        else:
            sanitized.append(MAX_LIDAR_RANGE_M)

    out = [MAX_LIDAR_RANGE_M] * 360

    for i in range(360):
        # Match CoVAPSy indexing convention while supporting arbitrary lidar resolution.
        src_pos = (float(i) * float(count) / 360.0) % float(count)
        lo = int(src_pos) % count
        hi = (lo + 1) % count
        frac = src_pos - lo
        distance = (1.0 - frac) * sanitized[-lo % count] + frac * sanitized[-hi % count]
        mapped_idx = (i - 180) % 360
        out[mapped_idx] = float(distance)

    return _median_filter_circular(out, LIDAR_MEDIAN_FILTER_RADIUS)


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


def find_free_gaps(mask):
    """Return all contiguous traversable segments as (start, end)."""
    gaps = []
    start = None

    for i, valid in enumerate(mask):
        if valid and start is None:
            start = i
        elif not valid and start is not None:
            gaps.append((start, i - 1))
            start = None

    if start is not None:
        gaps.append((start, len(mask) - 1))
    return gaps


def choose_best_gap(front_ranges, front_angles, gaps):
    """Score gaps by width + clearance and keep a mild forward-heading bias."""
    if not gaps:
        return None

    total = max(len(front_ranges), 1)
    max_heading = math.radians(FRONT_FOV_DEG)

    def score(gap):
        start, end = gap
        width = (end - start + 1) / float(total)
        section = front_ranges[start : end + 1]
        clearance = clamp(median_or_default(section, 0.0) / MAX_LIDAR_RANGE_M, 0.0, 1.0)
        center_idx = (start + end) // 2
        heading = 1.0 - clamp(abs(front_angles[center_idx]) / max(max_heading, 1e-6), 0.0, 1.0)
        return (
            GAP_SCORE_WIDTH_WEIGHT * width
            + GAP_SCORE_CLEARANCE_WEIGHT * clearance
            + GAP_SCORE_HEADING_WEIGHT * heading
        )

    return max(gaps, key=score)


def forward_clearance_for_heading(front_ranges, front_angles, heading_rad, half_window_deg):
    """Nearest obstacle distance around the current steering heading."""
    half_window_rad = math.radians(half_window_deg)
    nearest = MAX_LIDAR_RANGE_M
    for dist, ang in zip(front_ranges, front_angles):
        if abs(ang - heading_rad) <= half_window_rad:
            nearest = min(nearest, dist)
    return nearest


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


def _apply_nearest_obstacle_bubble(
    front_ranges,
    reference_ranges,
    front_angles,
    effective_safety_radius,
    max_span_rad=None,
):
    nearest_idx = min(
        range(len(reference_ranges)),
        key=lambda idx: reference_ranges[idx] + 0.30 * abs(front_angles[idx]),
    )
    nearest_dist = reference_ranges[nearest_idx]
    if nearest_dist < MAX_LIDAR_RANGE_M:
        nearest_angle = front_angles[nearest_idx]
        arc_scale = max(nearest_dist, MIN_VALID_RANGE_M)
        for i in range(len(front_ranges)):
            delta = abs(front_angles[i] - nearest_angle)
            if max_span_rad is not None and delta > max_span_rad:
                continue
            arc_dist = arc_scale * delta
            if arc_dist < effective_safety_radius:
                front_ranges[i] = 0.0

    return nearest_idx, nearest_dist


def _mask_gap_shoulders(front_ranges, front_angles):
    gap_limit_rad = math.radians(GAP_SELECTION_FOV_DEG)
    for i in range(len(front_ranges)):
        if abs(front_angles[i]) > gap_limit_rad:
            front_ranges[i] = 0.0


def _find_best_gap_target(front_ranges, front_angles):
    traversable = [r > MIN_VALID_RANGE_M for r in front_ranges]
    free_gaps = find_free_gaps(traversable)
    best_gap = choose_best_gap(front_ranges, front_angles, free_gaps)
    if best_gap is None:
        return None, None

    gap_start, gap_end = best_gap
    gap_center = 0.5 * (gap_start + gap_end)
    gap_half_span = max(0.5 * (gap_end - gap_start + 1), 1.0)
    best_local = max(
        range(gap_start, gap_end + 1),
        key=lambda idx: (
            front_ranges[idx]
            - ANGLE_PENALTY_PER_RAD * abs(front_angles[idx])
            - GAP_EDGE_PENALTY * abs((idx - gap_center) / gap_half_span)
        ),
    )
    return front_angles[best_local], best_gap


def _compute_far_center_steering(
    front_ranges,
    front_angles,
    camera_offset,
    far_center_gain,
    camera_center_gain,
    fusion_clearance_ref_m,
):
    left_clear, right_clear = side_clearances(front_ranges, front_angles)
    clearance_ref = max(float(fusion_clearance_ref_m), 0.1)
    lateral_balance = clamp((left_clear - right_clear) / clearance_ref, -1.0, 1.0)
    lidar_center = float(far_center_gain) * lateral_balance
    camera_center = float(camera_center_gain) * clamp(camera_offset, -1.0, 1.0)
    return clamp(lidar_center + camera_center, -MAX_STEERING_RAD, MAX_STEERING_RAD)


def _compute_close_far_blend_weight(
    forward_clearance,
    ttc_proxy,
    turn_urgency,
    far_weight_min,
    far_weight_max,
    fusion_clearance_ref_m,
    ttc_target_sec,
):
    lo = clamp(far_weight_min, 0.0, 1.0)
    hi = clamp(far_weight_max, 0.0, 1.0)
    if lo > hi:
        lo, hi = hi, lo

    clearance_score = clamp(forward_clearance / max(float(fusion_clearance_ref_m), 0.1), 0.0, 1.0)
    ttc_score = clamp(ttc_proxy / max(float(ttc_target_sec), 0.2), 0.0, 1.0)
    stability_score = 1.0 - clamp(turn_urgency, 0.0, 1.0)
    openness = (
        FUSION_CLEARANCE_WEIGHT * clearance_score
        + FUSION_TTC_WEIGHT * ttc_score
        + FUSION_STABILITY_WEIGHT * stability_score
    )
    return lo + (hi - lo) * openness


def traffic_mode_pass_margin(mode):
    value = str(mode).strip().lower()
    if value == "aggressive":
        return 0.14
    if value == "conservative":
        return 0.28
    return 0.20


def traffic_mode_speed_bias(mode):
    value = str(mode).strip().lower()
    if value == "aggressive":
        return 1.0
    if value == "conservative":
        return 0.82
    return 0.92


def select_passing_side(front_dist, left_clear, right_clear, pass_margin, follow_distance):
    if front_dist > follow_distance:
        return 0
    if (left_clear - right_clear) > pass_margin:
        return 1
    if (right_clear - left_clear) > pass_margin:
        return -1
    return 0


def compute_near_horizon_weight(
    base_weight,
    front_dist,
    clear_ref_dist,
    opponent_confidence,
    traffic_boost,
    clearance_boost,
    steer_disagreement_boost,
    near_steer,
    far_steer,
    max_steering,
    weight_min,
    weight_max,
):
    w_min = clamp(weight_min, 0.0, 1.0)
    w_max = clamp(weight_max, 0.0, 1.0)
    if w_min > w_max:
        w_min, w_max = w_max, w_min

    clear_ref = max(float(clear_ref_dist), 0.1)
    clearance_risk = 1.0 - clamp(front_dist / clear_ref, 0.0, 1.0)
    opp_risk = clamp(opponent_confidence, 0.0, 1.0)
    steer_span = max(abs(float(max_steering)), 1e-6)
    steer_disagreement = clamp(abs(float(near_steer) - float(far_steer)) / steer_span, 0.0, 1.0)
    raw = (
        float(base_weight)
        + float(traffic_boost) * opp_risk
        + float(clearance_boost) * clearance_risk
        + float(steer_disagreement_boost) * steer_disagreement
    )
    return clamp(raw, w_min, w_max)


def blend_near_far_command(near_speed, near_steer, far_speed, far_steer, near_weight):
    w = clamp(near_weight, 0.0, 1.0)
    speed = w * float(near_speed) + (1.0 - w) * float(far_speed)
    steer = w * float(near_steer) + (1.0 - w) * float(far_steer)
    return speed, steer


def compute_ttc_limited_speed(front_dist, desired_speed, target_ttc_sec):
    speed = max(0.0, float(desired_speed))
    ttc_target = max(float(target_ttc_sec), 0.2)
    ttc = float(front_dist) / max(speed, 0.05)
    if ttc < ttc_target:
        speed = min(speed, float(front_dist) / ttc_target)
    return max(0.0, speed)


def apply_rule_guards(speed_m_s, steering_rad, front_dist, left_clear, right_clear, max_speed_m_s):
    speed = clamp(speed_m_s, 0.0, max_speed_m_s)
    steer = clamp(steering_rad, -0.55, 0.55)

    if front_dist < 0.22:
        return 0.0, 0.0
    if front_dist < 0.75 and min(left_clear, right_clear) < 0.60:
        # Only dampen steering that points INTO the closer wall.
        # Positive steer = turn left; left_clear < right_clear = left wall is closer.
        steering_into_wall = (
            (steer > 0.0 and left_clear < right_clear - 0.10)
            or (steer < 0.0 and right_clear < left_clear - 0.10)
        )
        if steering_into_wall:
            steer *= 0.45
        speed *= 0.80
    if abs(steer) > 0.35 and speed > 1.6:
        speed = 1.6
    return max(0.0, speed), steer


def _scan_metrics(ranges, opponent_detect_range_m):
    _idx, front_ranges, front_angles = front_sector(ranges)
    if not front_ranges:
        return MAX_LIDAR_RANGE_M, MAX_LIDAR_RANGE_M, MAX_LIDAR_RANGE_M, 0.0

    front_min = front_clearance(ranges)
    left_clear, right_clear = side_clearances(front_ranges, front_angles)
    detect_range = max(float(opponent_detect_range_m), 0.4)
    near_front = [
        r
        for r, a in zip(front_ranges, front_angles)
        if abs(math.degrees(a)) < 65.0 and r < detect_range
    ]
    points_score = clamp(len(near_front) / 30.0, 0.0, 1.0)
    dist_score = clamp((detect_range - front_min) / detect_range, 0.0, 1.0)
    lidar_conf = clamp(0.55 * points_score + 0.45 * dist_score, 0.0, 1.0)
    return front_min, left_clear, right_clear, lidar_conf


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
    turn_factor = 1.0 - 0.35 * clamp(abs(steering_rad) / MAX_STEERING_RAD, 0.0, 1.0)
    min_speed = MIN_SPEED_FLOOR_M_S + (BASE_MIN_SPEED_M_S - MIN_SPEED_FLOOR_M_S) * distance_factor * turn_factor
    return clamp(min_speed, MIN_SPEED_FLOOR_M_S, BASE_MIN_SPEED_M_S)


def adaptive_safety_radius(front_ranges, front_angles):
    """Reduce safety radius when the passage is narrow to avoid blocking all paths."""
    left_clear, right_clear = side_clearances(front_ranges, front_angles)
    passage_width = left_clear + right_clear
    MIN_SAFETY = 0.08
    MAX_SAFETY = SAFETY_RADIUS_M
    NARROW_THRESHOLD = 0.90
    WIDE_THRESHOLD = 1.50
    if passage_width >= WIDE_THRESHOLD:
        return MAX_SAFETY
    if passage_width <= NARROW_THRESHOLD:
        return MIN_SAFETY
    t = (passage_width - NARROW_THRESHOLD) / (WIDE_THRESHOLD - NARROW_THRESHOLD)
    return MIN_SAFETY + t * (MAX_SAFETY - MIN_SAFETY)


def estimate_curvature_speed_limit(ranges, speed_cap_m_s):
    """Estimate upcoming curvature from LiDAR range asymmetry and limit speed."""
    # Use shared AI curvature estimation if available
    if AI_SPEED_AVAILABLE:
        sectors = extract_sector_ranges(ranges)
        curv = estimate_curvature(sectors)
        from racing_intelligence import predictive_brake_factor
        brake = predictive_brake_factor(sectors, curv)
        return max(speed_cap_m_s * brake, MIN_SPEED_FLOOR_M_S)

    left_ranges = []
    right_ranges = []
    for deg in range(5, CURVATURE_LOOKAHEAD_HALF_DEG + 1):
        left_ranges.append(ranges[deg % 360])
        right_ranges.append(ranges[(-deg) % 360])

    if not left_ranges or not right_ranges:
        return speed_cap_m_s

    left_avg = sum(left_ranges) / len(left_ranges)
    right_avg = sum(right_ranges) / len(right_ranges)
    total_avg = 0.5 * (left_avg + right_avg)

    if total_avg < 0.01:
        return speed_cap_m_s

    asymmetry = abs(left_avg - right_avg) / total_avg

    if asymmetry < CURVATURE_TRIGGER_RATIO:
        return speed_cap_m_s

    reduction = 1.0 - CURVATURE_SPEED_REDUCTION_FACTOR * clamp(
        (asymmetry - CURVATURE_TRIGGER_RATIO) / (1.0 - CURVATURE_TRIGGER_RATIO), 0.0, 1.0
    )
    return max(speed_cap_m_s * reduction, MIN_SPEED_FLOOR_M_S)


def escalating_recovery(ranges, recovery_count, last_direction):
    """Multi-strategy recovery that escalates with consecutive stalls.

    Returns (speed, steering, duration, new_direction).
    """
    _idx, front_ranges, front_angles = front_sector(ranges)
    left_clear, right_clear = side_clearances(front_ranges, front_angles)

    if recovery_count == 0:
        steer = compute_reverse_steering(ranges)
        # Use full lock when really close to the wall for more effective escape
        if min(left_clear, right_clear) < 0.35:
            direction = 1 if left_clear > right_clear else -1
            steer = direction * MAX_STEERING_RAD
        else:
            direction = 1 if steer > 0 else -1
        return REVERSE_SPEED_M_S, steer, REVERSE_STEPS, direction

    elif recovery_count == 1:
        direction = -last_direction if last_direction != 0 else (1 if left_clear > right_clear else -1)
        steer = direction * REVERSE_STEERING_MAX_RAD
        return REVERSE_SPEED_M_S, steer, REVERSE_STEPS, direction

    else:
        direction = -last_direction if last_direction != 0 else 1
        steer = direction * MAX_STEERING_RAD
        return REVERSE_SPEED_HARD_M_S, steer, REVERSE_STEPS_LONG, direction


def extract_camera_guidance(rgb_camera, depth_camera):
    if rgb_camera is None:
        return {
            "steering_rad": 0.0,
            "confidence": 0.0,
            "wrong_order": False,
            "order_confidence": 0.0,
        }

    image = rgb_camera.getImage()
    width = int(rgb_camera.getWidth())
    height = int(rgb_camera.getHeight())
    if image is None or width <= 0 or height <= 0:
        return {
            "steering_rad": 0.0,
            "confidence": 0.0,
            "wrong_order": False,
            "order_confidence": 0.0,
        }

    depth_image = None
    depth_w = 0
    depth_h = 0
    if depth_camera is not None:
        try:
            depth_image = depth_camera.getRangeImage()
            depth_w = int(depth_camera.getWidth())
            depth_h = int(depth_camera.getHeight())
            if depth_image is not None and len(depth_image) < (depth_w * depth_h):
                depth_image = None
        except Exception:
            depth_image = None

    roi_y0 = int(height * CAMERA_ROI_START_FRAC)
    center_x = 0.5 * width

    red_count = 0
    green_count = 0
    red_sum_x = 0.0
    green_sum_x = 0.0

    red_left = 0
    red_right = 0
    green_left = 0
    green_right = 0

    depth_left_samples = []
    depth_right_samples = []

    for y in range(roi_y0, height, CAMERA_SAMPLE_STRIDE_Y):
        for x in range(0, width, CAMERA_SAMPLE_STRIDE_X):
            red = Camera.imageGetRed(image, width, x, y)
            green = Camera.imageGetGreen(image, width, x, y)
            blue = Camera.imageGetBlue(image, width, x, y)

            cmax = max(red, green, blue)
            if cmax < CAMERA_MIN_BRIGHTNESS:
                continue
            cmin = min(red, green, blue)
            saturation = (cmax - cmin) / float(max(cmax, 1))
            if saturation < CAMERA_MIN_SATURATION:
                continue

            is_red = (
                red > CAMERA_RED_DOMINANCE * green
                and red > CAMERA_RED_DOMINANCE * blue
            )
            is_green = (
                green > CAMERA_GREEN_DOMINANCE * red
                and green > CAMERA_GREEN_DOMINANCE * blue
            )
            if not is_red and not is_green:
                continue

            on_left_half = x < center_x
            if is_red:
                red_count += 1
                red_sum_x += x
                if on_left_half:
                    red_left += 1
                else:
                    red_right += 1
            elif is_green:
                green_count += 1
                green_sum_x += x
                if on_left_half:
                    green_left += 1
                else:
                    green_right += 1

            if depth_image is not None and depth_w > 0 and depth_h > 0:
                depth_x = clamp(int(x * depth_w / width), 0, depth_w - 1)
                depth_y = clamp(int(y * depth_h / height), 0, depth_h - 1)
                depth_idx = depth_y * depth_w + depth_x
                depth_value = float(depth_image[depth_idx])
                if valid_depth_value(depth_value):
                    if on_left_half:
                        depth_left_samples.append(depth_value)
                    else:
                        depth_right_samples.append(depth_value)

    steer_lane = 0.0
    red_cx = None
    green_cx = None
    if red_count >= CAMERA_MIN_COLOR_PIXELS:
        red_cx = red_sum_x / red_count
    if green_count >= CAMERA_MIN_COLOR_PIXELS:
        green_cx = green_sum_x / green_count

    if red_cx is not None and green_cx is not None:
        lane_center = 0.5 * (red_cx + green_cx)
        lateral_error = (lane_center - center_x) / max(center_x, 1.0)
        # Positive steering in this controller means turning left.
        steer_lane = -CAMERA_LANE_GAIN_RAD * lateral_error
    elif green_cx is not None:
        # Seeing only right border (green): bias to the left.
        steer_lane = CAMERA_SINGLE_BORDER_STEER_RAD
    elif red_cx is not None:
        # Seeing only left border (red): bias to the right.
        steer_lane = -CAMERA_SINGLE_BORDER_STEER_RAD

    nearest_left = _quantile(depth_left_samples, CAMERA_DEPTH_PERCENTILE)
    nearest_right = _quantile(depth_right_samples, CAMERA_DEPTH_PERCENTILE)

    steer_depth = 0.0
    left_valid = nearest_left is not None
    right_valid = nearest_right is not None
    if left_valid and right_valid:
        side_imbalance = clamp(
            (nearest_right - nearest_left) / CAMERA_DEPTH_BALANCE_RANGE_M,
            -1.0,
            1.0,
        )
        # If left side is too close, steer right.
        steer_depth = -CAMERA_DEPTH_GAIN_RAD * side_imbalance
    elif left_valid:
        steer_depth = -0.7 * CAMERA_DEPTH_GAIN_RAD
    elif right_valid:
        steer_depth = 0.7 * CAMERA_DEPTH_GAIN_RAD

    steering_rad = clamp(steer_lane + steer_depth, -MAX_STEERING_RAD, MAX_STEERING_RAD)

    total_colored = red_count + green_count
    confidence = clamp(total_colored / CAMERA_CONFIDENCE_PIXELS, 0.0, 1.0)
    lane_locked = red_cx is not None and green_cx is not None
    if lane_locked:
        confidence = clamp(confidence + 0.25, 0.0, 1.0)
    else:
        confidence = min(confidence, CAMERA_SINGLE_BORDER_CONFIDENCE_CAP)

    wrong_order = False
    order_confidence = 0.0
    side_total = red_left + red_right + green_left + green_right
    if side_total > 0:
        expected_side_ratio = (red_left + green_right) / float(side_total)
        side_order_strength = abs((2.0 * expected_side_ratio) - 1.0)
        order_confidence = max(order_confidence, side_order_strength)
        if side_order_strength > 0.20:
            if EXPECTED_RED_ON_LEFT:
                wrong_order = expected_side_ratio < 0.5
            else:
                wrong_order = expected_side_ratio > 0.5
    if red_cx is not None and green_cx is not None:
        separation = abs(green_cx - red_cx)
        order_confidence = max(
            order_confidence,
            clamp(separation / max(width * 0.5, 1.0), 0.0, 1.0),
        )
        if separation >= CAMERA_ORDER_MIN_SEPARATION_PX:
            red_left_of_green = red_cx < green_cx
            wrong_order = red_left_of_green != EXPECTED_RED_ON_LEFT

    return {
        "steering_rad": steering_rad,
        "confidence": confidence,
        "wrong_order": wrong_order,
        "order_confidence": order_confidence,
        "lane_locked": lane_locked,
        "depth_sample_count": min(len(depth_left_samples), len(depth_right_samples)),
    }


def blend_lidar_and_camera_steering(lidar_steering_rad, camera_state):
    confidence = clamp(camera_state.get("confidence", 0.0), 0.0, 1.0)
    if confidence <= 0.0:
        return clamp(lidar_steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)

    camera_steering = clamp(
        camera_state.get("steering_rad", 0.0),
        -MAX_STEERING_RAD,
        MAX_STEERING_RAD,
    )
    blend = CAMERA_BLEND_MIN + (CAMERA_BLEND_MAX - CAMERA_BLEND_MIN) * confidence
    if not camera_state.get("lane_locked", False):
        blend *= CAMERA_SINGLE_BORDER_BLEND_SCALE
    if camera_state.get("depth_sample_count", 0) < CAMERA_MIN_DEPTH_SAMPLES:
        blend *= 0.8
    fused = (1.0 - blend) * lidar_steering_rad + blend * camera_steering

    if (
        camera_state.get("wrong_order", False)
        and camera_state.get("order_confidence", 0.0) > CAMERA_WRONG_ORDER_CONFIDENCE
    ):
        # When color order says direction is likely wrong, trust camera guidance more.
        fused = 0.45 * lidar_steering_rad + 0.55 * camera_steering

    return clamp(fused, -MAX_STEERING_RAD, MAX_STEERING_RAD)


def advanced_gap_follower(
    ranges,
    speed_cap_m_s,
    vehicle_state=None,
    camera_offset=0.0,
    use_close_far_fusion=DEFAULT_USE_CLOSE_FAR_FUSION,
    far_center_gain=DEFAULT_REACTIVE_FAR_CENTER_GAIN,
    camera_center_gain=DEFAULT_REACTIVE_CAMERA_CENTER_GAIN,
    far_weight_min=DEFAULT_REACTIVE_FAR_WEIGHT_MIN,
    far_weight_max=DEFAULT_REACTIVE_FAR_WEIGHT_MAX,
    fusion_clearance_ref_m=DEFAULT_REACTIVE_FUSION_CLEARANCE_REF_M,
    return_debug=False,
):
    global _ai_prev_speed
    _front_indices, front_ranges, front_angles = front_sector(ranges)
    if not front_ranges:
        if return_debug:
            return 0.0, 0.0, {"valid": False, "reason": "no_front_ranges"}
        return 0.0, 0.0

    reference_ranges = front_ranges.copy()

    # 1) Disparity extension (use reference_ranges to detect edges, avoiding cascade)
    half_width = CAR_WIDTH_M * 0.5
    angle_step = (2.0 * math.pi) / 360.0
    for i in range(1, len(front_ranges)):
        if abs(reference_ranges[i] - reference_ranges[i - 1]) > DISPARITY_THRESHOLD_M:
            closer_idx = i if reference_ranges[i] < reference_ranges[i - 1] else i - 1
            closer_range = reference_ranges[closer_idx]
            if closer_range > MIN_VALID_RANGE_M:
                extend_angle = math.atan2(half_width, closer_range)
                extend_count = int(extend_angle / angle_step)
                lo = max(0, closer_idx - extend_count)
                hi = min(len(front_ranges), closer_idx + extend_count + 1)
                for j in range(lo, hi):
                    front_ranges[j] = min(front_ranges[j], closer_range)

    front_after_disparity = front_ranges.copy()

    # 2) Safety bubble around nearest obstacle (adaptive radius in narrow passages).
    effective_safety_radius = adaptive_safety_radius(reference_ranges, front_angles)
    _nearest_idx, nearest_dist = _apply_nearest_obstacle_bubble(
        front_ranges=front_ranges,
        reference_ranges=reference_ranges,
        front_angles=front_angles,
        effective_safety_radius=effective_safety_radius,
    )
    _mask_gap_shoulders(front_ranges, front_angles)

    # 3) Best free gap (width + clearance score), with deterministic fallback bubble.
    target_angle, best_gap = _find_best_gap_target(front_ranges, front_angles)
    gap_fallback_used = False
    if best_gap is None:
        gap_fallback_used = True
        front_ranges = front_after_disparity.copy()
        fallback_safety_radius = max(
            effective_safety_radius * FALLBACK_BUBBLE_RADIUS_SCALE,
            FALLBACK_BUBBLE_MIN_RADIUS_M,
        )
        _apply_nearest_obstacle_bubble(
            front_ranges=front_ranges,
            reference_ranges=reference_ranges,
            front_angles=front_angles,
            effective_safety_radius=fallback_safety_radius,
            max_span_rad=math.radians(FALLBACK_BUBBLE_MAX_SPAN_DEG),
        )
        _mask_gap_shoulders(front_ranges, front_angles)
        target_angle, best_gap = _find_best_gap_target(front_ranges, front_angles)
        if best_gap is None:
            if return_debug:
                return 0.0, 0.0, {
                    "valid": False,
                    "reason": "no_gap_after_fallback",
                    "gap_fallback_used": gap_fallback_used,
                }
            return 0.0, 0.0

    # 4) Close/far steering synthesis.
    forward_center_dist = forward_clearance_for_heading(reference_ranges, front_angles, 0.0, 12)
    corner_bias = compute_corner_bias(reference_ranges, front_angles, forward_center_dist)
    close_steering = clamp(target_angle + corner_bias, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    close_steering = apply_turn_direction_sanity(close_steering, reference_ranges, front_angles)

    far_steering = close_steering
    far_weight = 0.0
    desired_steering = close_steering
    if use_close_far_fusion:
        far_steering = _compute_far_center_steering(
            front_ranges=reference_ranges,
            front_angles=front_angles,
            camera_offset=camera_offset,
            far_center_gain=far_center_gain,
            camera_center_gain=camera_center_gain,
            fusion_clearance_ref_m=fusion_clearance_ref_m,
        )
        ttc_proxy = forward_center_dist / max(abs(_ai_prev_speed), MIN_SPEED_FLOOR_M_S * 0.8, 0.05)
        turn_urgency = abs(close_steering) / max(MAX_STEERING_RAD, 1e-6)
        far_weight = _compute_close_far_blend_weight(
            forward_clearance=forward_center_dist,
            ttc_proxy=ttc_proxy,
            turn_urgency=turn_urgency,
            far_weight_min=far_weight_min,
            far_weight_max=far_weight_max,
            fusion_clearance_ref_m=fusion_clearance_ref_m,
            ttc_target_sec=SPEED_TARGET_TTC_SEC,
        )
        desired_steering = (
            (1.0 - far_weight) * close_steering + far_weight * far_steering
        )
        desired_steering = clamp(desired_steering, -MAX_STEERING_RAD, MAX_STEERING_RAD)

    # 5) Steering + speed adaptation
    steering_rad = desired_steering
    projected_clearance = forward_clearance_for_heading(
        reference_ranges,
        front_angles,
        steering_rad,
        SPEED_LOOKAHEAD_WINDOW_DEG,
    )
    projected_ttc_clearance = forward_clearance_for_heading(
        reference_ranges,
        front_angles,
        steering_rad,
        SPEED_TTC_WINDOW_DEG,
    )

    turn_ratio = clamp(abs(steering_rad) / MAX_STEERING_RAD, 0.0, 1.0)
    steering_factor = 1.0 - 0.45 * turn_ratio
    clearance_factor = clamp((projected_clearance - 0.08) / 1.2, 0.0, 1.0)

    left_clear, right_clear = side_clearances(reference_ranges, front_angles)
    adaptive_floor = adaptive_min_speed(projected_clearance, steering_rad)
    if projected_clearance < 0.45 and turn_ratio > 0.55:
        adaptive_floor = max(adaptive_floor, 0.22)
    # Ensure momentum during active cornering away from the wall
    if turn_ratio > 0.60:
        steering_away = (
            (steering_rad > 0.0 and left_clear >= right_clear)
            or (steering_rad < 0.0 and right_clear >= left_clear)
        )
        if steering_away:
            adaptive_floor = max(adaptive_floor, 0.25)
    cap = max(adaptive_floor, speed_cap_m_s)

    # Use AI-enhanced speed when available, fall back to classic heuristic
    if AI_SPEED_AVAILABLE:
        sectors = extract_sector_ranges(ranges)
        curvature = estimate_curvature(sectors)
        if vehicle_state is not None:
            curvature = fuse_curvature_with_imu(curvature, vehicle_state)
        if vehicle_state is not None:
            speed_m_s = compute_optimal_speed_fused(
                max_speed=cap,
                min_speed=adaptive_floor,
                steering_rad=steering_rad,
                max_steering=MAX_STEERING_RAD,
                projected_clearance=projected_clearance,
                passage_width=left_clear + right_clear,
                curvature=curvature,
                prev_speed=_ai_prev_speed,
                sector_ranges=sectors,
                ttc_clearance=projected_ttc_clearance,
                vehicle_state=vehicle_state,
                ttc_target_sec=SPEED_TARGET_TTC_SEC,
            )
        else:
            speed_m_s = compute_optimal_speed(
                max_speed=cap,
                min_speed=adaptive_floor,
                steering_rad=steering_rad,
                max_steering=MAX_STEERING_RAD,
                projected_clearance=projected_clearance,
                passage_width=left_clear + right_clear,
                curvature=curvature,
                prev_speed=_ai_prev_speed,
                sector_ranges=sectors,
                ttc_clearance=projected_ttc_clearance,
                ttc_target_sec=SPEED_TARGET_TTC_SEC,
            )
    else:
        speed_m_s = adaptive_floor + (cap - adaptive_floor) * steering_factor * clearance_factor
        speed_m_s = clamp(speed_m_s, adaptive_floor, cap)
        ttc_speed_cap = projected_ttc_clearance / max(SPEED_TARGET_TTC_SEC, 0.1)
        speed_m_s = min(speed_m_s, max(0.0, ttc_speed_cap))

    # Forward-direction safety guard
    actual_fwd = forward_clearance_for_heading(
        reference_ranges, front_angles, 0.0, SPEED_LOOKAHEAD_WINDOW_DEG,
    )
    if speed_m_s > 0.05 and actual_fwd < 2.0:
        fwd_ttc = actual_fwd / speed_m_s
        if fwd_ttc < FORWARD_TTC_MIN_SEC:
            speed_m_s = min(speed_m_s, max(adaptive_floor * 0.3, actual_fwd / FORWARD_TTC_MIN_SEC))
    if actual_fwd < EMERGENCY_WALL_DIST_M:
        speed_m_s = min(speed_m_s, adaptive_floor)

    _ai_prev_speed = speed_m_s
    if return_debug:
        return steering_rad, speed_m_s, {
            "valid": True,
            "close_steering_rad": close_steering,
            "far_steering_rad": far_steering,
            "far_weight": far_weight,
            "desired_steering_rad": steering_rad,
            "forward_clearance_m": actual_fwd,
            "projected_clearance_m": projected_clearance,
            "projected_ttc_clearance_m": projected_ttc_clearance,
            "left_clear_m": left_clear,
            "right_clear_m": right_clear,
            "gap_fallback_used": gap_fallback_used,
            "best_gap": best_gap,
        }
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
    close_hits = sum(1 for idx in front_window if ranges[idx] < EMERGENCY_BLOCK_DISTANCE_M)
    if close_hits < EMERGENCY_MIN_BLOCK_HITS:
        return False
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


def front_clearance(ranges, half_window_deg=12):
    return min(ranges[deg % 360] for deg in range(-half_window_deg, half_window_deg + 1))


def read_vehicle_speed_m_s(driver):
    # Driver API exposes km/h; convert to m/s when available.
    getter = getattr(driver, "getCurrentSpeed", None)
    if getter is None:
        return None
    try:
        return max(0.0, float(getter()) / 3.6)
    except Exception:
        return None


def compute_corner_bias(front_ranges, front_angles, nearest_dist):
    if nearest_dist >= CORNER_TRIGGER_DIST_M:
        return 0.0

    left_clear, right_clear = side_clearances(front_ranges, front_angles)
    clearance_delta = left_clear - right_clear
    direction = clamp(clearance_delta / CORNER_CLEARANCE_REF_M, -1.0, 1.0)
    severity = clamp((CORNER_TRIGGER_DIST_M - nearest_dist) / CORNER_TRIGGER_DIST_M, 0.0, 1.0)
    return CORNER_BIAS_MAX_RAD * severity * direction


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


def stabilize_steering_command(target_steering_rad, prev_steering_rad, forward_clearance=999.0):
    target_steering_rad = clamp(target_steering_rad, -MAX_STEERING_RAD, MAX_STEERING_RAD)
    delta = target_steering_rad - prev_steering_rad

    # Adaptive rate limit: faster when wall is close ahead
    effective_rate = STEERING_RATE_LIMIT_RAD
    if forward_clearance < SLEW_URGENCY_DIST_M:
        urgency = 1.0 - forward_clearance / SLEW_URGENCY_DIST_M
        effective_rate = STEERING_RATE_LIMIT_RAD * (1.0 + SLEW_URGENCY_BOOST * urgency)

    limited_delta = clamp(delta, -effective_rate, effective_rate)
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

    # Enable IMU (InertialUnit) and GPS for sensor fusion
    imu_device = None
    gps_device = None
    try:
        imu_device = InertialUnit("imu")
        imu_device.enable(sensor_time_step)
    except Exception:
        imu_device = None
    try:
        gps_device = GPS("gps")
        gps_device.enable(sensor_time_step)
    except Exception:
        gps_device = None

    # Initialize EKF state estimator
    state_estimator = None
    if STATE_ESTIMATOR_AVAILABLE and imu_device is not None:
        state_estimator = VehicleStateEstimator()

    rgb_camera, rgb_camera_name = try_enable_optional_sensor(
        driver,
        RGB_CAMERA_CANDIDATES,
        sensor_time_step,
    )
    depth_camera, depth_camera_name = try_enable_optional_sensor(
        driver,
        DEPTH_CAMERA_CANDIDATES,
        sensor_time_step,
    )

    use_camera_guidance = env_flag(CAMERA_GUIDANCE_ENV, default=True)
    use_depth_guidance = env_flag(DEPTH_GUIDANCE_ENV, default=True)
    if not use_camera_guidance:
        rgb_camera = None
        depth_camera = None
    elif not use_depth_guidance:
        depth_camera = None

    use_close_far_fusion = env_flag(USE_CLOSE_FAR_FUSION_ENV, default=DEFAULT_USE_CLOSE_FAR_FUSION)
    reactive_far_center_gain = env_float(REACTIVE_FAR_CENTER_GAIN_ENV, DEFAULT_REACTIVE_FAR_CENTER_GAIN)
    reactive_camera_center_gain = env_float(
        REACTIVE_CAMERA_CENTER_GAIN_ENV, DEFAULT_REACTIVE_CAMERA_CENTER_GAIN
    )
    reactive_far_weight_min = env_float(REACTIVE_FAR_WEIGHT_MIN_ENV, DEFAULT_REACTIVE_FAR_WEIGHT_MIN)
    reactive_far_weight_max = env_float(REACTIVE_FAR_WEIGHT_MAX_ENV, DEFAULT_REACTIVE_FAR_WEIGHT_MAX)
    reactive_fusion_clearance_ref_m = env_float(
        REACTIVE_FUSION_CLEARANCE_REF_ENV, DEFAULT_REACTIVE_FUSION_CLEARANCE_REF_M
    )

    traffic_mode = env_text(TRAFFIC_MODE_ENV, DEFAULT_TRAFFIC_MODE).strip().lower()
    opponent_detect_range_m = max(
        0.4,
        env_float(OPPONENT_DETECT_RANGE_ENV, DEFAULT_OPPONENT_DETECT_RANGE_M),
    )
    follow_distance_m = max(0.2, env_float(FOLLOW_DISTANCE_ENV, DEFAULT_FOLLOW_DISTANCE_M))
    near_weight_base = env_float(TACTICAL_NEAR_WEIGHT_BASE_ENV, DEFAULT_NEAR_WEIGHT_BASE)
    near_weight_min = env_float(TACTICAL_NEAR_WEIGHT_MIN_ENV, DEFAULT_NEAR_WEIGHT_MIN)
    near_weight_max = env_float(TACTICAL_NEAR_WEIGHT_MAX_ENV, DEFAULT_NEAR_WEIGHT_MAX)
    near_weight_clear_ref_dist = env_float(
        TACTICAL_NEAR_WEIGHT_CLEAR_REF_ENV, DEFAULT_NEAR_WEIGHT_CLEAR_REF_DIST
    )
    near_weight_traffic_boost = env_float(
        TACTICAL_NEAR_WEIGHT_TRAFFIC_BOOST_ENV, DEFAULT_NEAR_WEIGHT_TRAFFIC_BOOST
    )
    near_weight_clearance_boost = env_float(
        TACTICAL_NEAR_WEIGHT_CLEARANCE_BOOST_ENV, DEFAULT_NEAR_WEIGHT_CLEARANCE_BOOST
    )
    near_weight_steer_disagreement_boost = env_float(
        TACTICAL_NEAR_WEIGHT_STEER_DISAGREEMENT_BOOST_ENV,
        DEFAULT_NEAR_WEIGHT_STEER_DISAGREEMENT_BOOST,
    )
    pass_lock_sec = max(0.0, env_float(PASS_LOCK_SEC_ENV, DEFAULT_PASS_LOCK_SEC))

    keyboard = driver.getKeyboard()
    keyboard.enable(sensor_time_step)

    auto_mode = False
    use_advanced = True
    reverse_ticks = 0
    reverse_steering = 0.0
    reverse_speed_current = REVERSE_SPEED_M_S
    stuck_counter = 0
    no_progress_counter = 0
    low_speed_stall_counter = 0
    recovery_count = 0
    recovery_cooldown = 0
    last_recovery_direction = 0
    post_recovery_ticks = 0
    post_recovery_steer_sign = 0
    speed_cap = DEFAULT_MAX_SPEED_M_S
    smoothed_speed = 0.0
    prev_steering = 0.0
    last_ranges = [MAX_LIDAR_RANGE_M] * 360
    wrong_order_counter = 0
    uturn_phase = 0        # 0=inactive, 1=brake, 2=reverse+steer, 3=forward+steer
    uturn_ticks = 0
    uturn_steer_sign = 1.0  # +1 or -1: which direction to steer during U-turn
    logs_enabled = env_flag(STANDALONE_LOG_ENV, default=False)
    log_counter = 0
    prev_imu_speed = 0.0
    prev_imu_yaw = 0.0
    prev_imu_time = 0.0
    pass_side = 0
    pass_lock_until = 0.0

    print("=" * 64)
    print("COVAPSY Standalone Webots Controller")
    print("No ROS required. Click the 3D window then use:")
    print("  A=start  N=stop  S=algo toggle  R=reverse  +/- speed cap  L=logs")
    print(f"Active speed profile: {DEFAULT_RACE_PROFILE} (cap={speed_cap:.2f} m/s)")
    print(
        f"Runtime logs: {'ON' if logs_enabled else 'OFF'} "
        f"(toggle with L or env {STANDALONE_LOG_ENV}=1)"
    )
    if rgb_camera is not None:
        print(
            f"Camera guidance: RGB sensor enabled ({rgb_camera_name}) "
            f"[{CAMERA_GUIDANCE_ENV}=1]"
        )
    else:
        print(
            "Camera guidance: RGB sensor disabled/unavailable, LiDAR-only mode "
            f"[set {CAMERA_GUIDANCE_ENV}=1 to enable]"
        )
    if depth_camera is not None:
        print(
            f"Camera guidance: depth sensor enabled ({depth_camera_name}) "
            f"[{DEPTH_GUIDANCE_ENV}=1]"
        )
    else:
        print(
            "Camera guidance: depth sensor disabled/unavailable "
            f"[set {DEPTH_GUIDANCE_ENV}=1 to enable]"
        )
    if state_estimator is not None:
        print("IMU sensor fusion: ENABLED (EKF state estimator active)")
    else:
        print("IMU sensor fusion: DISABLED (no InertialUnit or estimator unavailable)")
    print(
        "Reactive close/far fusion: "
        f"{'ON' if use_close_far_fusion else 'OFF'} "
        f"(far_gain={reactive_far_center_gain:.2f}, cam_gain={reactive_camera_center_gain:.2f}, "
        f"w=[{reactive_far_weight_min:.2f},{reactive_far_weight_max:.2f}], "
        f"clear_ref={reactive_fusion_clearance_ref_m:.2f})"
    )
    print(
        "Standalone tactical blending: "
        f"traffic_mode={traffic_mode}, detect_range={opponent_detect_range_m:.2f}m, "
        f"follow={follow_distance_m:.2f}m, near_w=[{near_weight_min:.2f},{near_weight_max:.2f}]"
    )
    while driver.step() != -1:
        while True:
            key = keyboard.getKey()
            if key == -1:
                break
            if key in (ord("A"), ord("a")):
                auto_mode = True
                reverse_ticks = 0
                reverse_steering = 0.0
                reverse_speed_current = REVERSE_SPEED_M_S
                stuck_counter = 0
                no_progress_counter = 0
                low_speed_stall_counter = 0
                recovery_count = 0
                recovery_cooldown = 0
                last_recovery_direction = 0
                smoothed_speed = 0.0
                prev_steering = 0.0
                wrong_order_counter = 0
                uturn_phase = 0
                uturn_ticks = 0
                pass_side = 0
                pass_lock_until = 0.0
                print("[standalone] auto mode ON")
            elif key in (ord("N"), ord("n")):
                auto_mode = False
                reverse_ticks = 0
                reverse_steering = 0.0
                no_progress_counter = 0
                low_speed_stall_counter = 0
                smoothed_speed = 0.0
                prev_steering = 0.0
                wrong_order_counter = 0
                uturn_phase = 0
                uturn_ticks = 0
                pass_side = 0
                pass_lock_until = 0.0
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
                speed_cap = min(MAX_SPEED_LIMIT_M_S, speed_cap + 0.2)
                print(f"[standalone] speed_cap={speed_cap:.2f} m/s")
            elif key in (ord("-"), ord("_")):
                speed_cap = max(MIN_SPEED_FLOOR_M_S, speed_cap - 0.2)
                print(f"[standalone] speed_cap={speed_cap:.2f} m/s")
            elif key in (ord("L"), ord("l")):
                logs_enabled = not logs_enabled
                print(f"[standalone] runtime logs {'ON' if logs_enabled else 'OFF'}")

        if not auto_mode:
            continue

        ranges = read_lidar_front_zero(lidar)
        last_ranges = ranges

        # ---- IMU sensor fusion update ----
        vehicle_state = None
        if state_estimator is not None:
            cur_time = driver.getTime()
            # Read IMU (InertialUnit → roll/pitch/yaw)
            if imu_device is not None:
                rpy = imu_device.getRollPitchYaw()
                yaw = float(rpy[2])
                # Derive yaw rate from yaw delta
                dt_imu = max(cur_time - prev_imu_time, 0.001) if prev_imu_time > 0 else 0.032
                yaw_rate = (yaw - prev_imu_yaw) / dt_imu
                # Handle wrap-around at ±π
                if yaw_rate > math.pi / dt_imu:
                    yaw_rate -= 2.0 * math.pi / dt_imu
                elif yaw_rate < -math.pi / dt_imu:
                    yaw_rate += 2.0 * math.pi / dt_imu

                # Get speed from vehicle API
                cur_speed = read_vehicle_speed_m_s(driver)
                if cur_speed is None:
                    cur_speed = 0.0
                lon_accel = (cur_speed - prev_imu_speed) / dt_imu
                lat_accel = cur_speed * yaw_rate  # centripetal

                state_estimator.update_imu(yaw_rate, lat_accel, lon_accel)
                state_estimator.update_wheel_speed(cur_speed)
                state_estimator.update_yaw(yaw)
                vehicle_state = state_estimator.state

                prev_imu_speed = cur_speed
                prev_imu_yaw = yaw
                prev_imu_time = cur_time

        if reverse_ticks > 0:
            reverse_ticks -= 1
            no_progress_counter = 0
            low_speed_stall_counter = 0
            prev_steering = reverse_steering
            set_drive_command(driver, reverse_steering, reverse_speed_current)
            if reverse_ticks == 0:
                # Activate post-recovery steering override
                post_recovery_ticks = POST_RECOVERY_OVERRIDE_STEPS
                post_recovery_steer_sign = -last_recovery_direction if last_recovery_direction != 0 else 0
            continue

        # ---- U-turn maneuver execution ----
        if uturn_phase > 0:
            uturn_ticks -= 1
            if uturn_phase == 1:
                # Phase 1: brake to stop
                set_drive_command(driver, 0.0, 0.0)
                if uturn_ticks <= 0:
                    uturn_phase = 2
                    uturn_ticks = UTURN_REVERSE_STEPS
                    print("[standalone] U-turn phase 2: reverse with full lock")
            elif uturn_phase == 2:
                # Phase 2: reverse with full lock steering
                steer = UTURN_STEER_RAD * uturn_steer_sign
                prev_steering = steer
                set_drive_command(driver, steer, UTURN_REVERSE_SPEED_M_S)
                if uturn_ticks <= 0:
                    uturn_phase = 3
                    uturn_ticks = UTURN_FORWARD_STEPS
                    print("[standalone] U-turn phase 3: forward to complete turn")
            elif uturn_phase == 3:
                # Phase 3: forward with full lock to finish turning
                steer = UTURN_STEER_RAD * uturn_steer_sign
                prev_steering = steer
                set_drive_command(driver, steer, UTURN_FORWARD_SPEED_M_S)
                if uturn_ticks <= 0:
                    uturn_phase = 0
                    wrong_order_counter = 0
                    smoothed_speed = UTURN_FORWARD_SPEED_M_S
                    print("[standalone] U-turn complete, resuming normal driving")
            no_progress_counter = 0
            low_speed_stall_counter = 0
            continue

        if emergency_front_blocked(ranges):
            stuck_counter += 1
            if stuck_counter > EMERGENCY_STUCK_STEPS:
                rev_speed, rev_steer, rev_duration, rev_dir = escalating_recovery(
                    ranges, recovery_count, last_recovery_direction
                )
                reverse_ticks = rev_duration
                reverse_steering = rev_steer
                reverse_speed_current = rev_speed
                last_recovery_direction = rev_dir
                recovery_count += 1
                recovery_cooldown = RECOVERY_ESCALATION_WINDOW_STEPS
                stuck_counter = 0
                no_progress_counter = 0
                low_speed_stall_counter = 0
                print(
                    f"[standalone] emergency reverse (front blocked, strategy={recovery_count - 1}) "
                    f"steer={math.degrees(rev_steer):.1f} deg "
                    f"speed={rev_speed:.2f} duration={rev_duration}"
                )
            set_drive_command(driver, 0.0, 0.0)
            continue

        stuck_counter = max(0, stuck_counter - 1)

        # Decay recovery escalation cooldown
        if recovery_cooldown > 0:
            recovery_cooldown -= 1
            if recovery_cooldown == 0:
                recovery_count = 0

        # Curvature-aware speed limiting (only applied to fallback path;
        # AI path has its own curvature handling to avoid double-penalty)
        if AI_SPEED_AVAILABLE:
            effective_cap = speed_cap
        else:
            curvature_cap = estimate_curvature_speed_limit(ranges, speed_cap)
            effective_cap = min(speed_cap, curvature_cap)

        camera_state = extract_camera_guidance(rgb_camera, depth_camera)
        camera_offset = float(camera_state.get("steering_rad", 0.0))

        reactive_debug = {}
        if use_advanced:
            steering, target_speed, reactive_debug = advanced_gap_follower(
                ranges,
                effective_cap,
                vehicle_state=vehicle_state,
                camera_offset=camera_offset,
                use_close_far_fusion=use_close_far_fusion,
                far_center_gain=reactive_far_center_gain,
                camera_center_gain=reactive_camera_center_gain,
                far_weight_min=reactive_far_weight_min,
                far_weight_max=reactive_far_weight_max,
                fusion_clearance_ref_m=reactive_fusion_clearance_ref_m,
                return_debug=True,
            )
        else:
            steering, target_speed = simple_baseline(ranges, effective_cap)
            reactive_debug = {
                "valid": True,
                "close_steering_rad": steering,
                "far_steering_rad": steering,
                "far_weight": 0.0,
                "desired_steering_rad": steering,
                "gap_fallback_used": False,
            }

        # Keep legacy camera blend as additional near-horizon correction.
        steering = blend_lidar_and_camera_steering(steering, camera_state)

        # Tactical near/far blend under traffic pressure.
        front_min, left_clear, right_clear, lidar_conf = _scan_metrics(
            ranges,
            opponent_detect_range_m=opponent_detect_range_m,
        )
        camera_conf = clamp(1.0 - abs(camera_offset), 0.0, 1.0)
        confidence = clamp((0.8 * lidar_conf) + (0.2 * camera_conf), 0.0, 1.0)

        near_speed = float(target_speed)
        near_steer = float(steering)
        far_speed = float(target_speed)
        far_steer = float(reactive_debug.get("far_steering_rad", steering))
        near_weight = compute_near_horizon_weight(
            base_weight=near_weight_base,
            front_dist=front_min,
            clear_ref_dist=near_weight_clear_ref_dist,
            opponent_confidence=confidence,
            traffic_boost=near_weight_traffic_boost,
            clearance_boost=near_weight_clearance_boost,
            steer_disagreement_boost=near_weight_steer_disagreement_boost,
            near_steer=near_steer,
            far_steer=far_steer,
            max_steering=MAX_STEERING_RAD,
            weight_min=near_weight_min,
            weight_max=near_weight_max,
        )
        base_speed, base_steer = blend_near_far_command(
            near_speed=near_speed,
            near_steer=near_steer,
            far_speed=far_speed,
            far_steer=far_steer,
            near_weight=near_weight,
        )

        pass_margin = traffic_mode_pass_margin(traffic_mode)
        side = select_passing_side(
            front_dist=front_min,
            left_clear=left_clear,
            right_clear=right_clear,
            pass_margin=pass_margin,
            follow_distance=follow_distance_m,
        )
        now_sec = driver.getTime()
        if now_sec < pass_lock_until and pass_side != 0:
            side = pass_side
        elif side != 0:
            pass_side = side
            pass_lock_until = now_sec + pass_lock_sec
        else:
            pass_side = 0

        target_speed = clamp(base_speed, 0.0, effective_cap) * traffic_mode_speed_bias(traffic_mode)
        steering = clamp(base_steer, -MAX_STEERING_RAD, MAX_STEERING_RAD)
        if confidence > OPPONENT_CONFIDENCE_MIN and front_min < follow_distance_m:
            slow_factor = clamp(front_min / max(follow_distance_m, 0.2), 0.30, 1.0)
            target_speed *= slow_factor
            steering += DEFAULT_STEERING_BIAS_GAIN * float(side)
            steering = clamp(steering, -MAX_STEERING_RAD, MAX_STEERING_RAD)
        else:
            target_speed = min(target_speed, effective_cap)

        target_speed = compute_ttc_limited_speed(
            front_dist=front_min,
            desired_speed=target_speed,
            target_ttc_sec=DEFAULT_TACTICAL_TARGET_TTC_SEC,
        )
        target_speed, steering = apply_rule_guards(
            speed_m_s=target_speed,
            steering_rad=steering,
            front_dist=front_min,
            left_clear=left_clear,
            right_clear=right_clear,
            max_speed_m_s=effective_cap,
        )

        # Wrong-direction detection → U-turn trigger
        if (
            camera_state["wrong_order"]
            and camera_state["order_confidence"] > UTURN_CONFIDENCE_THRESHOLD
        ):
            wrong_order_counter += 1
            # Slow down while evaluating
            target_speed = min(target_speed, CAMERA_WRONG_ORDER_SPEED_CAP_M_S)
            if wrong_order_counter >= UTURN_TRIGGER_FRAMES:
                # Determine steer direction: pick the side with more open space
                left_clear = sum(ranges[i] for i in range(0, 90)) / 90.0
                right_clear = sum(ranges[i] for i in range(270, 360)) / 90.0
                uturn_steer_sign = 1.0 if left_clear >= right_clear else -1.0
                uturn_phase = 1
                uturn_ticks = UTURN_BRAKE_STEPS
                smoothed_speed = 0.0
                print(
                    f"[standalone] U-turn triggered! "
                    f"(wrong_order for {wrong_order_counter} frames, "
                    f"confidence={camera_state['order_confidence']:.2f}, "
                    f"steer_sign={uturn_steer_sign:+.0f})"
                )
                set_drive_command(driver, 0.0, 0.0)
                continue
        else:
            wrong_order_counter = max(0, wrong_order_counter - 2)

        front_dist = front_min
        steering = stabilize_steering_command(steering, prev_steering, front_dist)
        prev_steering = steering

        # Post-recovery steering override: bias away from the wall for a few ticks
        if post_recovery_ticks > 0:
            post_recovery_ticks -= 1
            override_mag = MAX_STEERING_RAD * 0.6  # moderate counter-steer
            steering = clamp(
                steering + post_recovery_steer_sign * override_mag,
                -MAX_STEERING_RAD, MAX_STEERING_RAD,
            )

        if target_speed > smoothed_speed:
            smoothed_speed = min(target_speed, smoothed_speed + SPEED_RAMP_UP_M_S)
        else:
            decel_rate = SPEED_RAMP_DOWN_M_S
            if front_dist < 0.40:
                decel_rate = SPEED_RAMP_DOWN_M_S * 2.0
            smoothed_speed = max(target_speed, smoothed_speed - decel_rate)

        actual_speed = read_vehicle_speed_m_s(driver)
        speed_log_value = actual_speed if actual_speed is not None else -1.0

        if (
            smoothed_speed > NO_PROGRESS_SPEED_CMD_M_S
            and actual_speed is not None
            and actual_speed < NO_PROGRESS_SPEED_ACTUAL_M_S
            and front_dist < NO_PROGRESS_FRONT_DIST_M
        ):
            no_progress_counter += 1
        else:
            no_progress_counter = max(0, no_progress_counter - 1)

        if target_speed <= (MIN_SPEED_FLOOR_M_S + 0.02) and front_dist < EARLY_STALL_WINDOW_DIST_M:
            low_speed_stall_counter += 1
        else:
            low_speed_stall_counter = max(0, low_speed_stall_counter - 1)

        # Rapid re-stall: if still stuck right after recovery, escalate immediately
        rapid_restall = (
            post_recovery_ticks == 0
            and recovery_cooldown > (RECOVERY_ESCALATION_WINDOW_STEPS - POST_RECOVERY_OVERRIDE_STEPS - 5)
            and front_dist < 0.35
            and actual_speed is not None
            and actual_speed < 0.08
        )

        if (
            no_progress_counter >= NO_PROGRESS_TRIGGER_STEPS
            or low_speed_stall_counter >= EARLY_STALL_TRIGGER_STEPS
            or rapid_restall
        ):
            rev_speed, rev_steer, rev_duration, rev_dir = escalating_recovery(
                ranges, recovery_count, last_recovery_direction
            )
            reverse_ticks = rev_duration
            reverse_steering = rev_steer
            reverse_speed_current = rev_speed
            last_recovery_direction = rev_dir
            recovery_count += 1
            recovery_cooldown = RECOVERY_ESCALATION_WINDOW_STEPS
            no_progress_counter = 0
            low_speed_stall_counter = 0
            print(
                f"[standalone] stall recovery (strategy={recovery_count - 1}) "
                f"(front={front_dist:.2f}m, speed={speed_log_value:.2f}m/s, "
                f"steer={math.degrees(rev_steer):.1f} deg, "
                f"rev_speed={rev_speed:.2f}, duration={rev_duration})"
            )
            continue

        if logs_enabled:
            log_counter += 1
            if log_counter % STANDALONE_LOG_EVERY_STEPS == 0:
                print(
                    "[standalone][log] "
                    f"mode={'ADV' if use_advanced else 'BASE'} "
                    f"spd={smoothed_speed:.2f}/{target_speed:.2f}m/s "
                    f"cap={speed_cap:.2f} "
                    f"steer={math.degrees(steering):+.1f}deg "
                    f"front={front_dist:.2f}m "
                    f"v={speed_log_value:.2f}m/s "
                    f"stall={no_progress_counter}/{low_speed_stall_counter} "
                    f"cam_conf={camera_state['confidence']:.2f} "
                    f"cam_wrong={int(camera_state['wrong_order'])} "
                    f"cam_order={camera_state['order_confidence']:.2f} "
                    f"uturn_cnt={wrong_order_counter}/{UTURN_TRIGGER_FRAMES} "
                    f"cam_lock={int(camera_state.get('lane_locked', False))} "
                    f"cam_depth={camera_state.get('depth_sample_count', 0)} "
                    f"opp={confidence:.2f} "
                    f"nw={near_weight:.2f} "
                    f"side={side:+d} "
                    f"far_w={reactive_debug.get('far_weight', 0.0):.2f} "
                    f"gap_fb={int(bool(reactive_debug.get('gap_fallback_used', False)))} "
                    f"imu={'Y' if vehicle_state else 'N'}"
                    + (
                        f" yr={vehicle_state.yaw_rate:+.2f}r/s"
                        f" lg={vehicle_state.cornering_g:.2f}g"
                        f" fc={vehicle_state.abs_curvature_normalised:.2f}"
                        if vehicle_state else ""
                    )
                )

        set_drive_command(driver, steering, smoothed_speed)


if __name__ == "__main__":
    run_controller()
