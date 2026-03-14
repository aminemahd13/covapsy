"""Pure mode-selection and safety rules for final drive command."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


def should_trigger_wall_deadlock(
    mode_active: bool,
    has_recent_scan: bool,
    has_recent_reactive: bool,
    min_front_dist: float,
    stuck_front_blocked_dist: float,
    left_clearance: float,
    right_clearance: float,
    stuck_side_blocked_dist: float,
    stuck_side_asymmetry_min: float,
    reactive_steer: float,
    stuck_front_blocked_steer_min: float,
    speed_known: bool,
    actual_speed: float,
    stuck_actual_speed_max: float,
    selected_linear_x: float,
    stuck_cmd_speed_min: float,
) -> bool:
    """Return True when wall contact causes zero-speed deadlock."""
    if not mode_active or not has_recent_scan or not has_recent_reactive:
        return False
    if not math.isfinite(float(reactive_steer)):
        return False

    front_blocked = (
        math.isfinite(float(min_front_dist))
        and float(min_front_dist) <= float(stuck_front_blocked_dist)
    )
    side_min = min(float(left_clearance), float(right_clearance))
    side_asym = abs(float(left_clearance) - float(right_clearance))
    side_blocked = (
        math.isfinite(side_min)
        and side_min <= float(stuck_side_blocked_dist)
        and side_asym >= float(stuck_side_asymmetry_min)
    )
    has_contact = front_blocked or side_blocked

    steer_intent = abs(float(reactive_steer)) >= float(stuck_front_blocked_steer_min)
    selected_stopped = float(selected_linear_x) <= float(stuck_cmd_speed_min)

    # Primary path: wall contact + steering intent + reactive output is stopped
    if has_contact and steer_intent and selected_stopped:
        if not speed_known:
            return True
        if not math.isfinite(float(actual_speed)):
            return False
        return float(actual_speed) < float(stuck_actual_speed_max)

    # Oblique stuck: side wall very close, reactive still commanding forward
    # (wall outside its forward cone), but actual speed is zero.
    # This catches the 45° stuck scenario.
    oblique_stuck = (
        side_blocked
        and side_min <= 0.15
        and not selected_stopped
        and speed_known
        and math.isfinite(float(actual_speed))
        and float(actual_speed) < float(stuck_actual_speed_max)
    )
    if oblique_stuck:
        return True

    return False


def select_recovery_timeout(
    stuck_timeout: float,
    stuck_imu_timeout_factor: float,
    stuck_front_blocked_timeout_factor: float,
    stuck_deadlock_unknown_speed_timeout_factor: float,
    is_spinning: bool,
    is_impact: bool,
    wall_deadlock: bool,
    speed_known: bool,
    position_stuck: bool = False,
) -> float:
    """Select recovery timeout while preserving trigger precedence."""
    base_timeout = max(0.01, float(stuck_timeout))
    if is_spinning or is_impact:
        factor = float(stuck_imu_timeout_factor)
    elif position_stuck:
        factor = 0.5
    elif wall_deadlock and speed_known:
        factor = float(stuck_front_blocked_timeout_factor)
    elif wall_deadlock and not speed_known:
        factor = float(stuck_deadlock_unknown_speed_timeout_factor)
    else:
        factor = 1.0
    return base_timeout * max(0.05, factor)


def select_mode_command(
    mode: str,
    reactive_cmd: DriveCommand,
    pursuit_cmd: DriveCommand,
    tactical_cmd: DriveCommand | None,
    tactical_enabled: bool,
    mapping_speed_cap: float,
    min_front_dist: float,
    emergency_stop_distance: float = 0.15,
    pursuit_min_speed: float = 0.01,
    tactical_min_speed: float = 0.02,
    tactical_max_abs_steer: float = 0.55,
) -> DriveCommand:
    """Select final command from mode state and safety constraints."""
    cmd = DriveCommand()
    current_mode = str(mode).upper()

    if current_mode in ("IDLE", "STOPPED"):
        cmd = DriveCommand()
    elif current_mode in ("MAPPING", "LEARNING"):
        cmd = DriveCommand(
            linear_x=min(float(reactive_cmd.linear_x), float(mapping_speed_cap)),
            angular_z=float(reactive_cmd.angular_z),
        )
    elif current_mode == "REACTIVE":
        cmd = DriveCommand(
            linear_x=float(reactive_cmd.linear_x),
            angular_z=float(reactive_cmd.angular_z),
        )
    elif current_mode == "RACING":
        tactical_valid = (
            tactical_enabled
            and tactical_cmd is not None
            and math.isfinite(float(tactical_cmd.linear_x))
            and math.isfinite(float(tactical_cmd.angular_z))
            and float(tactical_cmd.linear_x) >= tactical_min_speed
            and abs(float(tactical_cmd.angular_z)) <= tactical_max_abs_steer
        )
        if tactical_valid:
            source = tactical_cmd
        else:
            source = pursuit_cmd if abs(float(pursuit_cmd.linear_x)) > pursuit_min_speed else reactive_cmd
        cmd = DriveCommand(
            linear_x=float(source.linear_x),
            angular_z=float(source.angular_z),
        )

    if float(min_front_dist) < float(emergency_stop_distance) and cmd.linear_x > 0.0:
        return DriveCommand()
    return cmd


def select_recovery_command(
    phase: int,
    reverse_speed: float,
    reverse_steer: float,
    rear_blocked: bool,
) -> DriveCommand:
    """Return drive command for ESC reverse recovery (Tamiya TBLE-02S protocol).

    Phase 0: brake pulse (large negative speed, zero steer) — arms ESC reverse.
    Phase 1: neutral (zero speed) — required pause before reverse engages.
    Phase 2: actual reverse with steering (aborted if rear is blocked).
    """
    if phase == 0:
        return DriveCommand(linear_x=-2.0, angular_z=0.0)
    if phase == 1:
        return DriveCommand(0.0, 0.0)
    if phase == 2:
        if rear_blocked:
            return DriveCommand(0.0, 0.0)
        return DriveCommand(linear_x=float(reverse_speed), angular_z=float(reverse_steer))
    return DriveCommand()
