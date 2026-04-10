#!/usr/bin/env python3
"""Shared steering policy helpers for bridge nodes."""

from __future__ import annotations

from typing import Tuple


def apply_external_steering_mode(steer_rad: float, external_steering_mode: bool) -> float:
    """Force STM32 steering neutral when external steering is active."""
    if external_steering_mode:
        return 0.0
    return float(steer_rad)


def steer_rad_to_goal_tick(
    *,
    steer_rad: float,
    max_steer_rad: float,
    center_tick: int,
    left_tick: int,
    right_tick: int,
) -> int:
    """Map steering angle (rad) to DYNAMIXEL goal position tick."""
    if max_steer_rad <= 1e-6:
        return int(center_tick)

    steer = max(-max_steer_rad, min(max_steer_rad, float(steer_rad)))
    if steer >= 0.0:
        span = float(left_tick - center_tick)
        goal = float(center_tick) + (steer / max_steer_rad) * span
    else:
        span = float(center_tick - right_tick)
        goal = float(center_tick) + (steer / max_steer_rad) * span

    lo = min(int(left_tick), int(right_tick), int(center_tick))
    hi = max(int(left_tick), int(right_tick), int(center_tick))
    return max(lo, min(hi, int(round(goal))))


def evaluate_steering_gate(
    *,
    started: bool,
    cmd_age_s: float,
    watchdog_timeout_s: float,
    emergency_brake: bool,
) -> Tuple[bool, bool, bool]:
    """
    Evaluate steering run state.

    Returns: (run_enable, watchdog_brake, wait_start)
    """
    if not started:
        return False, False, True
    if cmd_age_s > watchdog_timeout_s:
        return False, True, False
    if emergency_brake:
        return False, True, False
    return True, False, False
