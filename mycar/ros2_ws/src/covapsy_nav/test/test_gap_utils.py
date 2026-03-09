import numpy as np

from covapsy_nav.gap_utils import _choose_best_gap
from covapsy_nav.gap_utils import compute_gap_command


def _default_kwargs():
    return dict(
        angle_min=-np.pi,
        angle_increment=2.0 * np.pi / 360.0,
        car_width=0.30,
        max_speed=2.0,
        min_speed=0.5,
        max_range=5.0,
        safety_radius=0.20,
        disparity_threshold=0.5,
        steering_gain=1.0,
        fov_degrees=200.0,
    )


def test_gap_command_stops_when_no_gap():
    ranges = np.zeros(360, dtype=np.float32)
    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    assert speed == 0.0
    assert steering == 0.0


def test_gap_command_returns_finite_drive_for_open_track():
    ranges = np.full(360, 4.0, dtype=np.float32)
    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    assert 0.5 <= speed <= 2.0
    assert -0.5 <= steering <= 0.5


def test_gap_command_applies_steering_slew_limit():
    ranges = np.full(360, 4.0, dtype=np.float32)
    speed, steering = compute_gap_command(
        ranges_in=ranges,
        prev_steering=0.0,
        steering_slew_rate=0.03,
        **_default_kwargs(),
    )
    assert 0.0 <= speed <= 2.0
    assert abs(steering) <= 0.03 + 1e-9


def test_gap_command_avoids_side_wall_speed_collapse():
    ranges = np.full(360, 3.8, dtype=np.float32)
    # Close obstacle on side should not force crawl speed if projected heading is clear.
    ranges[82:113] = 0.12
    speed, _steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    assert speed >= 0.5


def test_gap_command_prefers_clearer_gap_direction():
    f_ranges = np.array([0.8] * 7 + [2.2] * 5, dtype=np.float32)
    f_angles = np.array(
        [-0.60, -0.50, -0.40, -0.30, -0.20, -0.10, -0.05, 0.05, 0.10, 0.20, 0.30, 0.40],
        dtype=np.float32,
    )
    gaps = [(0, 6), (7, 11)]

    best = _choose_best_gap(f_ranges=f_ranges, f_angles=f_angles, gaps=gaps, max_range=5.0)
    assert best == (7, 11)


# ---- Forward-direction safety tests ----


def test_wall_ahead_limits_speed():
    """When a wall is directly ahead, speed must drop even if side gaps exist."""
    ranges = np.full(360, 4.0, dtype=np.float32)
    # Wall directly ahead: angle_min=-π, increment=2π/360, so front = index 180
    for i in range(170, 191):
        ranges[i] = 0.22  # very close wall ahead

    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    # Speed should be severely limited due to forward wall (emergency brake threshold 0.30m)
    assert speed <= 0.6


def test_open_track_no_forward_penalty():
    """On wide-open track, forward safety should not reduce speed."""
    ranges = np.full(360, 4.5, dtype=np.float32)
    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    # Forward safety must not drag speed to a crawl on open track
    assert speed >= 0.5


def test_adaptive_slew_rate_near_wall():
    """With wall close ahead, steering can change more than base slew rate."""
    ranges = np.full(360, 4.0, dtype=np.float32)
    # Wall ahead: indices 170-190 (front ±10°)
    for i in range(170, 191):
        ranges[i] = 0.28

    base_slew = 0.04
    _, steer = compute_gap_command(
        ranges_in=ranges, prev_steering=0.0, steering_slew_rate=base_slew, **_default_kwargs(),
    )
    # Forward clearance 0.28m < urgency threshold 0.55m → adaptive boost kicks in
    # Effective slew > base → steering can exceed base_slew from prev_steering=0
    assert abs(steer) > base_slew + 0.001
