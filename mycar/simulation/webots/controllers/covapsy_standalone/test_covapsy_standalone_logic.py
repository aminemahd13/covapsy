import importlib.util
import math
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parent / "covapsy_standalone.py"
SPEC = importlib.util.spec_from_file_location("covapsy_standalone_module", MODULE_PATH)
covapsy_standalone = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(covapsy_standalone)


def make_ranges(default_dist=None):
    value = covapsy_standalone.MAX_LIDAR_RANGE_M if default_dist is None else float(default_dist)
    return [value] * 360


def set_sector(ranges, start_deg, end_deg, distance_m):
    for deg in range(start_deg, end_deg + 1):
        ranges[deg % 360] = float(distance_m)


class _FakeLidar:
    def __init__(self, raw):
        self._raw = raw

    def getRangeImage(self):
        return self._raw


def _ackermann_inner_angle(center_angle_rad: float) -> float:
    center_abs = abs(float(center_angle_rad))
    if center_abs < 1e-12:
        return 0.0
    wheelbase = float(covapsy_standalone.WEBOTS_WHEELBASE_M)
    track = float(covapsy_standalone.WEBOTS_TRACK_FRONT_M)
    radius_center = wheelbase / math.tan(center_abs)
    radius_inner = max(radius_center - (0.5 * track), 1e-9)
    return math.atan(wheelbase / radius_inner)


def test_max_steering_keeps_ackermann_inner_wheel_within_webots_limit():
    inner_angle = _ackermann_inner_angle(covapsy_standalone.MAX_STEERING_RAD)
    assert inner_angle <= covapsy_standalone.WEBOTS_STEERING_INNER_LIMIT_RAD + 1e-12


def test_uturn_full_lock_does_not_exceed_max_steering_cap():
    assert covapsy_standalone.UTURN_STEER_RAD <= covapsy_standalone.MAX_STEERING_RAD


def test_symmetric_corridor_prefers_near_center():
    ranges = make_ranges()
    set_sector(ranges, 70, 100, 0.55)
    set_sector(ranges, -100, -70, 0.55)

    steering, speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    assert abs(math.degrees(steering)) <= 4.0
    assert covapsy_standalone.MIN_SPEED_FLOOR_M_S <= speed <= covapsy_standalone.DEFAULT_MAX_SPEED_M_S


def test_left_tighter_than_right_yields_right_turn():
    ranges = make_ranges()
    set_sector(ranges, -12, 12, 0.30)
    set_sector(ranges, 35, 90, 0.15)
    set_sector(ranges, -90, -35, 1.80)

    steering, _speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    assert steering < 0.0


def test_right_tighter_than_left_yields_left_turn():
    ranges = make_ranges()
    set_sector(ranges, -12, 12, 0.30)
    set_sector(ranges, -90, -35, 0.15)
    set_sector(ranges, 35, 90, 1.80)

    steering, _speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    assert steering > 0.0


def test_no_traversable_gap_returns_stop():
    ranges = make_ranges()
    set_sector(
        ranges,
        -covapsy_standalone.FRONT_FOV_DEG,
        covapsy_standalone.FRONT_FOV_DEG,
        0.01,
    )

    steering, speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    assert steering == 0.0
    assert speed == 0.0


def test_sudden_side_swap_is_rate_limited():
    ranges_left_tight = make_ranges()
    set_sector(ranges_left_tight, 35, 90, 0.28)
    set_sector(ranges_left_tight, -90, -35, 1.80)

    ranges_right_tight = make_ranges()
    set_sector(ranges_right_tight, -90, -35, 0.28)
    set_sector(ranges_right_tight, 35, 90, 1.80)

    raw_left, _ = covapsy_standalone.advanced_gap_follower(
        ranges_left_tight,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )
    raw_right, _ = covapsy_standalone.advanced_gap_follower(
        ranges_right_tight,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    cmd_1 = covapsy_standalone.stabilize_steering_command(raw_left, 0.0)
    cmd_2 = covapsy_standalone.stabilize_steering_command(raw_right, cmd_1)

    assert abs(cmd_2 - cmd_1) <= covapsy_standalone.STEERING_RATE_LIMIT_RAD + 1e-12
    assert cmd_1 * cmd_2 >= 0.0


def test_close_side_wall_keeps_reasonable_speed_when_forward_is_clear():
    ranges = make_ranges(3.8)
    set_sector(ranges, 82, 112, 0.12)

    _steering, speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    assert speed >= 0.5


def test_gap_selection_prefers_clearer_gap_over_only_wider_gap():
    front_ranges = [0.8] * 7 + [2.2] * 5
    front_angles = [
        -0.60,
        -0.50,
        -0.40,
        -0.30,
        -0.20,
        -0.10,
        -0.05,
        0.05,
        0.10,
        0.20,
        0.30,
        0.40,
    ]
    gaps = [(0, 6), (7, 11)]

    chosen = covapsy_standalone.choose_best_gap(front_ranges, front_angles, gaps)

    assert chosen == (7, 11)


def test_far_center_steering_uses_side_clearance_balance():
    front_ranges = [0.8, 0.9, 1.0, 1.2, 1.8, 1.9, 2.0]
    front_angles = [math.radians(v) for v in (-70, -50, -30, 0, 30, 50, 70)]
    steer = covapsy_standalone._compute_far_center_steering(
        front_ranges=front_ranges,
        front_angles=front_angles,
        camera_offset=0.0,
        far_center_gain=0.4,
        camera_center_gain=0.0,
        fusion_clearance_ref_m=1.2,
    )
    assert steer > 0.0


def test_close_far_blend_weight_prefers_far_when_open():
    open_weight = covapsy_standalone._compute_close_far_blend_weight(
        forward_clearance=2.6,
        ttc_proxy=3.0,
        turn_urgency=0.1,
        far_weight_min=0.10,
        far_weight_max=0.60,
        fusion_clearance_ref_m=1.8,
        ttc_target_sec=1.2,
    )
    constrained_weight = covapsy_standalone._compute_close_far_blend_weight(
        forward_clearance=0.45,
        ttc_proxy=0.35,
        turn_urgency=0.85,
        far_weight_min=0.10,
        far_weight_max=0.60,
        fusion_clearance_ref_m=1.8,
        ttc_target_sec=1.2,
    )
    assert 0.10 <= constrained_weight <= 0.60
    assert 0.10 <= open_weight <= 0.60
    assert open_weight > constrained_weight


def test_close_strategy_stays_dominant_under_low_clearance():
    ranges = make_ranges(3.5)
    set_sector(ranges, -12, 12, 0.35)
    set_sector(ranges, 205, 260, 0.20)

    close_steer, close_speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
        use_close_far_fusion=False,
        camera_offset=1.0,
        far_center_gain=0.6,
        camera_center_gain=0.5,
    )
    fused_steer, fused_speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
        use_close_far_fusion=True,
        camera_offset=1.0,
        far_center_gain=0.6,
        camera_center_gain=0.5,
        far_weight_min=0.05,
        far_weight_max=0.80,
        fusion_clearance_ref_m=1.8,
    )

    assert close_speed >= 0.0
    assert fused_speed >= 0.0
    assert abs(close_steer) > 1e-4
    assert close_steer * fused_steer > 0.0
    assert abs(fused_steer - close_steer) < 0.18


def test_near_weight_increases_with_opponent_confidence():
    low_conf = covapsy_standalone.compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=1.5,
        clear_ref_dist=1.8,
        opponent_confidence=0.1,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.05,
        far_steer=0.05,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    high_conf = covapsy_standalone.compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=1.5,
        clear_ref_dist=1.8,
        opponent_confidence=0.9,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.05,
        far_steer=0.05,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    assert high_conf > low_conf


def test_near_weight_increases_with_steer_disagreement():
    aligned = covapsy_standalone.compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=1.5,
        clear_ref_dist=1.8,
        opponent_confidence=0.3,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.05,
        far_steer=0.05,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    disagree = covapsy_standalone.compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=1.5,
        clear_ref_dist=1.8,
        opponent_confidence=0.3,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.45,
        far_steer=-0.45,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    assert disagree > aligned


def test_adaptive_safety_radius_narrow_passage():
    """In a narrow passage (~0.85m), safety radius should shrink."""
    ranges = make_ranges()
    set_sector(ranges, 40, 80, 0.42)
    set_sector(ranges, -80, -40, 0.43)
    _idx, front_ranges, front_angles = covapsy_standalone.front_sector(ranges)
    radius = covapsy_standalone.adaptive_safety_radius(front_ranges, front_angles)
    assert radius < covapsy_standalone.SAFETY_RADIUS_M


def test_adaptive_safety_radius_wide_passage():
    """In a wide passage (>1.5m), full safety radius should be used."""
    ranges = make_ranges()
    set_sector(ranges, 40, 80, 1.2)
    set_sector(ranges, -80, -40, 1.2)
    _idx, front_ranges, front_angles = covapsy_standalone.front_sector(ranges)
    radius = covapsy_standalone.adaptive_safety_radius(front_ranges, front_angles)
    assert radius == covapsy_standalone.SAFETY_RADIUS_M


def test_curvature_speed_limit_straight():
    """Symmetric ranges (straight) should not reduce speed."""
    ranges = make_ranges(2.0)
    cap = 2.5
    result = covapsy_standalone.estimate_curvature_speed_limit(ranges, cap)
    assert result == cap


def test_curvature_speed_limit_curve():
    """Asymmetric ranges (curve) should reduce speed."""
    ranges = make_ranges(2.0)
    set_sector(ranges, 5, 40, 0.6)  # left side short = left curve
    cap = 2.5
    result = covapsy_standalone.estimate_curvature_speed_limit(ranges, cap)
    assert result < cap


def test_escalating_recovery_strategy_zero_keeps_computed_steer_near_wall(monkeypatch):
    """Strategy 0 should keep compute_reverse_steering output, even near walls."""
    ranges = make_ranges()
    set_sector(ranges, 40, 80, 0.20)   # left side close
    set_sector(ranges, -80, -40, 1.20)  # right side open

    monkeypatch.setattr(covapsy_standalone, "compute_reverse_steering", lambda _ranges: 0.20)

    _speed, steer_0, _dur_0, dir_0 = covapsy_standalone.escalating_recovery(ranges, 0, 0)

    assert math.isclose(steer_0, 0.20, rel_tol=0.0, abs_tol=1e-12)
    assert dir_0 == 1


def test_escalating_recovery_alternates_direction():
    """Consecutive recoveries should alternate direction and escalate steering cap."""
    ranges = make_ranges()
    set_sector(ranges, -10, 10, 0.20)

    _, steer_0, dur_0, dir_0 = covapsy_standalone.escalating_recovery(ranges, 0, 0)
    _, steer_1, dur_1, dir_1 = covapsy_standalone.escalating_recovery(ranges, 1, dir_0)
    _, steer_2, dur_2, dir_2 = covapsy_standalone.escalating_recovery(ranges, 2, dir_1)

    # Direction alternates
    assert dir_1 == -dir_0
    assert dir_2 == -dir_1
    # Strategy 1 uses REVERSE_STEERING_MAX_RAD (not full lock)
    assert math.isclose(abs(steer_1), covapsy_standalone.REVERSE_STEERING_MAX_RAD, rel_tol=0.0, abs_tol=1e-12)
    # Strategy 2+ remains full lock fallback
    assert math.isclose(abs(steer_2), covapsy_standalone.MAX_STEERING_RAD, rel_tol=0.0, abs_tol=1e-12)
    # Duration escalates at strategy 2
    assert dur_2 > dur_0
    # All have non-zero steering
    assert steer_0 != 0.0
    assert steer_1 != 0.0
    assert steer_2 != 0.0


def test_wider_fov_finds_gap_in_tight_curve():
    """With wider FOV, a gap in a tight curve should be found and steered toward."""
    ranges = make_ranges()
    # Simulate a tight left curve with track walls on both sides
    set_sector(ranges, -30, 60, 0.03)    # front blocked (approaching wall)
    set_sector(ranges, -100, -40, 0.35)  # right track border (inner wall)
    set_sector(ranges, 70, 100, 2.0)     # exit gap to the left (curve exit)

    steering, speed = covapsy_standalone.advanced_gap_follower(
        ranges,
        covapsy_standalone.DEFAULT_MAX_SPEED_M_S,
    )

    # Should find the gap and steer toward it (positive = left)
    assert steering > 0.0
    assert speed > 0.0


def test_read_lidar_front_zero_handles_non_360_resolution():
    raw = [0.6 + (0.2 * math.sin(i * 0.01)) for i in range(720)]
    lidar = _FakeLidar(raw)
    ranges = covapsy_standalone.read_lidar_front_zero(lidar)

    assert len(ranges) == 360
    assert all(covapsy_standalone.MIN_VALID_RANGE_M <= r <= covapsy_standalone.MAX_LIDAR_RANGE_M for r in ranges)


def test_emergency_front_blocked_requires_multiple_hits():
    ranges = make_ranges(2.0)
    ranges[0] = 0.10  # single noisy spike should not trigger emergency
    assert covapsy_standalone.emergency_front_blocked(ranges) is False

    set_sector(ranges, -2, 2, 0.10)  # several close points in front should trigger
    assert covapsy_standalone.emergency_front_blocked(ranges) is True


def test_camera_blend_reduced_without_lane_lock():
    lidar_steer = 0.0
    camera_state_unlocked = {
        "steering_rad": 0.4,
        "confidence": 1.0,
        "wrong_order": False,
        "order_confidence": 0.0,
        "lane_locked": False,
        "depth_sample_count": 0,
    }
    camera_state_locked = {
        "steering_rad": 0.4,
        "confidence": 1.0,
        "wrong_order": False,
        "order_confidence": 0.0,
        "lane_locked": True,
        "depth_sample_count": covapsy_standalone.CAMERA_MIN_DEPTH_SAMPLES,
    }

    unlocked = covapsy_standalone.blend_lidar_and_camera_steering(lidar_steer, camera_state_unlocked)
    locked = covapsy_standalone.blend_lidar_and_camera_steering(lidar_steer, camera_state_locked)
    assert abs(unlocked) < abs(locked)
