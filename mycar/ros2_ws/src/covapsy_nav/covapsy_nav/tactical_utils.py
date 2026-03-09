"""Pure tactical racing helpers used by the tactical race node."""

from __future__ import annotations

import math


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def select_passing_side(
    front_dist: float,
    left_clear: float,
    right_clear: float,
    pass_margin: float,
    follow_distance: float,
) -> int:
    """Return pass side: +1 left, -1 right, 0 hold."""
    if front_dist > follow_distance:
        return 0

    if (left_clear - right_clear) > pass_margin:
        return 1
    if (right_clear - left_clear) > pass_margin:
        return -1
    return 0


def fused_opponent_confidence(lidar_conf: float, camera_conf: float, camera_weight: float) -> float:
    """Fuse opponent confidence with optional camera confirmation."""
    lw = 1.0 - clamp(camera_weight, 0.0, 0.8)
    cw = 1.0 - lw
    return clamp((lw * lidar_conf) + (cw * camera_conf), 0.0, 1.0)


def compute_near_horizon_weight(
    base_weight: float,
    front_dist: float,
    clear_ref_dist: float,
    opponent_confidence: float,
    traffic_boost: float,
    clearance_boost: float,
    steer_disagreement_boost: float,
    near_steer: float,
    far_steer: float,
    max_steering: float,
    weight_min: float,
    weight_max: float,
) -> float:
    """Compute dynamic blend weight for near-horizon command influence."""
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


def blend_near_far_command(
    near_speed: float,
    near_steer: float,
    far_speed: float,
    far_steer: float,
    near_weight: float,
) -> tuple[float, float]:
    """Blend near and far drive targets with weight in [0..1]."""
    w = clamp(near_weight, 0.0, 1.0)
    speed = w * float(near_speed) + (1.0 - w) * float(far_speed)
    steer = w * float(near_steer) + (1.0 - w) * float(far_steer)
    return speed, steer


def apply_rule_guards(
    speed_m_s: float,
    steering_rad: float,
    front_dist: float,
    left_clear: float,
    right_clear: float,
    max_speed_m_s: float,
) -> tuple[float, float]:
    """Apply race-safety guardrails to tactical commands."""
    speed = clamp(speed_m_s, 0.0, max_speed_m_s)
    steer = clamp(steering_rad, -0.55, 0.55)

    # Hard stop in immediate near-collision distance.
    if front_dist < 0.22:
        return 0.0, 0.0

    # Avoid aggressive blocking/weaving when boxed in.
    if front_dist < 0.75 and min(left_clear, right_clear) < 0.60:
        steer *= 0.45
        speed *= 0.65

    # Wrong-way proxy guard: no reverse in racing tactical command.
    speed = max(0.0, speed)

    # Keep high steering only at moderated speed.
    if abs(steer) > 0.35 and speed > 1.6:
        speed = 1.6

    return speed, steer


def compute_ttc_limited_speed(front_dist: float, desired_speed: float, target_ttc_sec: float) -> float:
    """Reduce speed when estimated time-to-collision gets too low."""
    speed = max(0.0, float(desired_speed))
    ttc_target = max(float(target_ttc_sec), 0.2)
    ttc = float(front_dist) / max(speed, 0.05)
    if ttc < ttc_target:
        speed = min(speed, float(front_dist) / ttc_target)
    return max(0.0, speed)


def traffic_mode_pass_margin(mode: str) -> float:
    m = str(mode).strip().lower()
    if m == "aggressive":
        return 0.14
    if m == "conservative":
        return 0.28
    return 0.20


def traffic_mode_speed_bias(mode: str) -> float:
    m = str(mode).strip().lower()
    if m == "aggressive":
        return 1.0
    if m == "conservative":
        return 0.82
    return 0.92
