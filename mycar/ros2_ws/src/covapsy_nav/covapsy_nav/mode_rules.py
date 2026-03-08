"""Pure mode-selection and safety rules for final drive command."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


def select_mode_command(
    mode: str,
    reactive_cmd: DriveCommand,
    pursuit_cmd: DriveCommand,
    mapping_speed_cap: float,
    min_front_dist: float,
    emergency_stop_distance: float = 0.15,
    pursuit_min_speed: float = 0.01,
) -> DriveCommand:
    """Select final command from mode state and safety constraints."""
    cmd = DriveCommand()
    current_mode = str(mode).upper()

    if current_mode in ("IDLE", "STOPPED"):
        cmd = DriveCommand()
    elif current_mode == "MAPPING":
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

