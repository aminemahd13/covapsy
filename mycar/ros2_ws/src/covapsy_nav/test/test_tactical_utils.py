from covapsy_nav.tactical_utils import apply_rule_guards
from covapsy_nav.tactical_utils import compute_ttc_limited_speed
from covapsy_nav.tactical_utils import select_passing_side


def test_select_passing_side_prefers_left_when_clearer():
    side = select_passing_side(front_dist=0.7, left_clear=1.8, right_clear=0.9, pass_margin=0.2, follow_distance=1.1)
    assert side == 1


def test_select_passing_side_prefers_right_when_clearer():
    side = select_passing_side(front_dist=0.7, left_clear=0.9, right_clear=1.8, pass_margin=0.2, follow_distance=1.1)
    assert side == -1


def test_select_passing_side_holds_without_clear_advantage():
    side = select_passing_side(front_dist=0.7, left_clear=1.1, right_clear=1.0, pass_margin=0.2, follow_distance=1.1)
    assert side == 0


def test_rule_guard_blocks_emergency_collision():
    speed, steer = apply_rule_guards(
        speed_m_s=1.0,
        steering_rad=0.2,
        front_dist=0.15,
        left_clear=1.0,
        right_clear=1.0,
        max_speed_m_s=2.0,
    )
    assert speed == 0.0
    assert steer == 0.0


def test_ttc_limiter_reduces_speed():
    speed = compute_ttc_limited_speed(front_dist=0.5, desired_speed=2.0, target_ttc_sec=1.0)
    assert speed <= 0.5
