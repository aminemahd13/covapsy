"""Unit tests for the follow-the-gap driver.

Run: python -m pytest tests/test_gap_follower.py -v
"""

import math
import os
import sys

# shared module is in ../common
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common"))

import gap_follower as gf

MAXR = gf.DEFAULT_PARAMS["max_range_m"]


def scan(default=MAXR):
    """360-element scan, all-open by default."""
    return [default] * 360


def put(s, deg, dist):
    """Set the beam at car-angle deg (0=front, +left, -right)."""
    s[deg % 360] = dist


def block_arc(s, center_deg, half_width_deg, dist):
    for d in range(center_deg - half_width_deg, center_deg + half_width_deg + 1):
        s[d % 360] = dist


def test_open_field_goes_straight_and_fast():
    steer, speed = gf.gap_follow(scan())
    assert abs(steer) < 2.0, f"should aim straight, got {steer}"
    assert speed > 0.5, f"open field should be near max speed, got {speed}"


def test_wall_on_right_steers_left():
    s = scan()
    block_arc(s, -60, 25, 0.4)   # wall on the right
    steer, speed = gf.gap_follow(s)
    assert steer > 0.0, f"wall on right -> steer left (positive), got {steer}"
    assert speed > 0.0


def test_wall_on_left_steers_right():
    s = scan()
    block_arc(s, 60, 25, 0.4)    # wall on the left
    steer, speed = gf.gap_follow(s)
    assert steer < 0.0, f"wall on left -> steer right (negative), got {steer}"


def test_boxed_front_rear_clear_reverses():
    s = scan(0.25)               # everything close, boxed in front
    steer, speed = gf.gap_follow(s, rear_blocked=False)
    assert speed < 0.0, f"blocked & rear clear -> reverse, got speed {speed}"


def test_boxed_front_and_rear_stops():
    s = scan(0.25)
    steer, speed = gf.gap_follow(s, rear_blocked=True)
    assert speed == 0.0 and steer == 0.0, \
        f"blocked both ways -> full stop, got ({steer},{speed})"


def test_front_wall_only_with_open_sides_does_not_crash_forward():
    s = scan()
    block_arc(s, 0, 12, 0.2)     # wall dead ahead, sides and rear open
    steer, speed = gf.gap_follow(s, rear_blocked=False)
    # front blocked, rear clear: reverse instead of driving forward
    assert speed <= 0.0, f"wall ahead should not produce forward speed, got {speed}"


def test_speed_never_exceeds_cap():
    cap = gf.DEFAULT_PARAMS["max_speed_mps"]
    steer, speed = gf.gap_follow(scan())
    assert speed <= cap + 1e-9


def test_steer_clamped():
    s = scan()
    block_arc(s, -90, 40, 0.3)   # force a hard left turn
    steer, speed = gf.gap_follow(s)
    limit = gf.DEFAULT_PARAMS["max_steer_deg"]
    assert -limit - 1e-9 <= steer <= limit + 1e-9


def test_handles_zeros_and_nan_as_no_return():
    s = scan()
    s[10] = 0.0
    s[20] = float("nan")
    s[30] = float("inf")
    steer, speed = gf.gap_follow(s)   # must not raise
    assert speed > 0.0


def test_narrow_gap_between_two_walls_aims_center():
    s = scan()
    # two walls leaving a corridor straight ahead
    block_arc(s, 70, 30, 0.6)
    block_arc(s, -70, 30, 0.6)
    steer, speed = gf.gap_follow(s)
    assert abs(steer) < 10.0, f"corridor ahead -> mostly straight, got {steer}"
    assert speed > 0.0
