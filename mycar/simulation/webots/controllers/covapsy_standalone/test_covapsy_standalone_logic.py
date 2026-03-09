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
