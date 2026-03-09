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
