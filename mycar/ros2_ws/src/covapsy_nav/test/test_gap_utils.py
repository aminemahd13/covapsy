import numpy as np

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
    # Steering value is irrelevant at zero speed


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


# ---- Forward-direction safety tests ----


def test_wall_ahead_limits_speed():
    """When a wall is directly ahead, speed must drop even if side gaps exist."""
    ranges = np.full(360, 4.0, dtype=np.float32)
    # Wall directly ahead: angle_min=-pi, increment=2pi/360, so front = index 180
    for i in range(170, 191):
        ranges[i] = 0.22  # very close wall ahead

    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    # Speed should be severely limited due to forward wall
    assert speed <= 0.6


def test_open_track_no_forward_penalty():
    """On wide-open track, forward safety should not reduce speed."""
    ranges = np.full(360, 4.5, dtype=np.float32)
    speed, steering = compute_gap_command(ranges_in=ranges, **_default_kwargs())
    assert speed >= 0.5


def test_wall_ahead_triggers_escape_steering():
    """With wall close ahead and open side, steering must turn away from wall."""
    ranges = np.full(360, 4.0, dtype=np.float32)
    # Wall ahead: indices 170-190 (front ±10°)
    for i in range(170, 191):
        ranges[i] = 0.5

    _, steer = compute_gap_command(
        ranges_in=ranges, prev_steering=0.0, steering_slew_rate=0.25, **_default_kwargs(),
    )
    # Wall-ahead escape override should produce significant steering
    assert abs(steer) > 0.05


def test_wall_ahead_steers_toward_open_side():
    """Car should turn toward the open side when wall is ahead."""
    ranges = np.full(360, 0.5, dtype=np.float32)
    # angle_min=-pi, increment=2pi/360
    # Index 0 = -pi, index 180 = 0 (front), index 270 = pi/2 (left)
    # Open on LEFT side (positive angles = left)
    for i in range(200, 280):  # angles ~0.35 to ~1.75 rad (left side)
        ranges[i] = 4.0

    _, steer_left_open = compute_gap_command(
        ranges_in=ranges, prev_steering=0.0, steering_slew_rate=0.25, **_default_kwargs(),
    )

    # Open on RIGHT side (negative angles = right)
    ranges2 = np.full(360, 0.5, dtype=np.float32)
    for i in range(80, 160):  # angles ~-1.75 to ~-0.35 rad (right side)
        ranges2[i] = 4.0

    _, steer_right_open = compute_gap_command(
        ranges_in=ranges2, prev_steering=0.0, steering_slew_rate=0.25, **_default_kwargs(),
    )

    # Should steer left when left is open, right when right is open
    assert steer_left_open > 0.05, f"Should steer left, got {steer_left_open}"
    assert steer_right_open < -0.05, f"Should steer right, got {steer_right_open}"


def test_corridor_centering():
    """Car should steer away from a close side wall toward the open side."""
    ranges = np.full(360, 4.0, dtype=np.float32)
    # Close wall on the right side only
    for i in range(100, 160):  # right side
        ranges[i] = 0.4

    _, steer = compute_gap_command(
        ranges_in=ranges, prev_steering=0.0, steering_slew_rate=0.25, **_default_kwargs(),
    )
    # Should steer left (positive) away from right wall
    assert steer > 0.0


def test_symmetric_corridor_goes_straight():
    """In a symmetric corridor, steering should be near zero."""
    n = 360
    angles = -np.pi + np.arange(n) * (2 * np.pi / n)
    ranges = np.full(n, 5.0, dtype=np.float32)

    # Symmetric walls at ±45° to ±90°
    for i in range(n):
        a = angles[i]
        if 0.5 < abs(a) < 1.6:
            ranges[i] = 1.0

    speed, steer = compute_gap_command(
        ranges_in=ranges, prev_steering=0.0, steering_slew_rate=0.25, **_default_kwargs(),
    )
    assert abs(steer) < 0.1, f"Symmetric corridor should be straight, got {steer}"
    assert speed > 0.3
