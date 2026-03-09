from covapsy_nav.tactical_utils import apply_rule_guards
from covapsy_nav.tactical_utils import blend_near_far_command
from covapsy_nav.tactical_utils import compute_near_horizon_weight
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


def test_blend_near_far_command_hits_endpoints():
    speed_far, steer_far = blend_near_far_command(
        near_speed=1.0,
        near_steer=0.30,
        far_speed=2.0,
        far_steer=-0.10,
        near_weight=0.0,
    )
    speed_near, steer_near = blend_near_far_command(
        near_speed=1.0,
        near_steer=0.30,
        far_speed=2.0,
        far_steer=-0.10,
        near_weight=1.0,
    )
    assert speed_far == 2.0
    assert steer_far == -0.10
    assert speed_near == 1.0
    assert steer_near == 0.30


def test_near_weight_increases_with_opponent_confidence():
    low_conf = compute_near_horizon_weight(
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
    high_conf = compute_near_horizon_weight(
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


def test_near_weight_increases_when_front_clearance_is_low():
    open_track = compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=2.0,
        clear_ref_dist=1.8,
        opponent_confidence=0.2,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.02,
        far_steer=0.02,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    constrained = compute_near_horizon_weight(
        base_weight=0.35,
        front_dist=0.4,
        clear_ref_dist=1.8,
        opponent_confidence=0.2,
        traffic_boost=0.35,
        clearance_boost=0.30,
        steer_disagreement_boost=0.20,
        near_steer=0.02,
        far_steer=0.02,
        max_steering=0.5,
        weight_min=0.2,
        weight_max=0.9,
    )
    assert constrained > open_track
