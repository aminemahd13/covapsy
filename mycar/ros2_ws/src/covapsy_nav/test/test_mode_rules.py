from covapsy_nav.mode_rules import DriveCommand
from covapsy_nav.mode_rules import select_recovery_timeout
from covapsy_nav.mode_rules import select_mode_command
from covapsy_nav.mode_rules import select_recovery_command
from covapsy_nav.mode_rules import should_trigger_wall_deadlock


def _call(mode, reactive, pursuit, mapping_speed_cap=0.5, min_front_dist=10.0, tactical=None, tactical_enabled=False):
    return select_mode_command(
        mode,
        reactive,
        pursuit,
        tactical_cmd=tactical,
        tactical_enabled=tactical_enabled,
        mapping_speed_cap=mapping_speed_cap,
        min_front_dist=min_front_dist,
    )


def _wall_deadlock_kwargs():
    return dict(
        mode_active=True,
        has_recent_scan=True,
        has_recent_reactive=True,
        min_front_dist=0.20,
        stuck_front_blocked_dist=0.22,
        left_clearance=0.60,
        right_clearance=0.55,
        stuck_side_blocked_dist=0.20,
        stuck_side_asymmetry_min=0.12,
        reactive_steer=0.10,
        stuck_front_blocked_steer_min=0.08,
        speed_known=True,
        actual_speed=0.02,
        stuck_actual_speed_max=0.06,
        selected_linear_x=0.0,
        stuck_cmd_speed_min=0.08,
    )


def test_idle_and_stopped_force_zero_output():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.2)
    pursuit = DriveCommand(linear_x=1.5, angular_z=-0.1)

    idle = _call("IDLE", reactive, pursuit)
    stopped = _call("STOPPED", reactive, pursuit)

    assert idle == DriveCommand()
    assert stopped == DriveCommand()


def test_mapping_caps_forward_speed():
    reactive = DriveCommand(linear_x=1.3, angular_z=0.15)
    cmd = _call("MAPPING", reactive, DriveCommand(), mapping_speed_cap=0.5, min_front_dist=10.0)
    assert cmd.linear_x == 0.5
    assert cmd.angular_z == reactive.angular_z


def test_racing_falls_back_to_reactive_when_pursuit_unavailable():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.25)
    pursuit = DriveCommand(linear_x=0.0, angular_z=-0.2)
    cmd = _call("RACING", reactive, pursuit, min_front_dist=10.0)
    assert cmd == reactive


def test_racing_prefers_tactical_when_enabled_and_valid():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.2)
    pursuit = DriveCommand(linear_x=1.5, angular_z=0.1)
    tactical = DriveCommand(linear_x=1.2, angular_z=-0.3)
    cmd = _call("RACING", reactive, pursuit, tactical=tactical, tactical_enabled=True)
    assert cmd == tactical


def test_racing_ignores_invalid_tactical_and_uses_pursuit():
    reactive = DriveCommand(linear_x=1.0, angular_z=0.2)
    pursuit = DriveCommand(linear_x=1.5, angular_z=0.1)
    tactical = DriveCommand(linear_x=1.2, angular_z=1.5)
    cmd = _call("RACING", reactive, pursuit, tactical=tactical, tactical_enabled=True)
    assert cmd == pursuit


def test_front_obstacle_forces_stop_for_forward_motion():
    reactive = DriveCommand(linear_x=0.8, angular_z=0.1)
    cmd = _call("REACTIVE", reactive, DriveCommand(), min_front_dist=0.1)
    assert cmd == DriveCommand()


def test_recovery_phase_zero_sends_brake_pulse():
    cmd = select_recovery_command(
        phase=0,
        reverse_speed=-0.4,
        reverse_steer=0.3,
        rear_blocked=False,
    )
    assert cmd == DriveCommand(linear_x=-2.0, angular_z=0.0)


def test_recovery_phase_one_sends_neutral():
    cmd = select_recovery_command(
        phase=1,
        reverse_speed=-0.4,
        reverse_steer=0.3,
        rear_blocked=False,
    )
    assert cmd == DriveCommand(linear_x=0.0, angular_z=0.0)


def test_recovery_phase_two_sends_reverse_command():
    cmd = select_recovery_command(
        phase=2,
        reverse_speed=-0.4,
        reverse_steer=0.3,
        rear_blocked=False,
    )
    assert cmd == DriveCommand(linear_x=-0.4, angular_z=0.3)


def test_recovery_phase_two_aborts_when_rear_blocked():
    cmd = select_recovery_command(
        phase=2,
        reverse_speed=-0.4,
        reverse_steer=0.3,
        rear_blocked=True,
    )
    assert cmd == DriveCommand(linear_x=0.0, angular_z=0.0)


def test_recovery_invalid_phase_returns_zero():
    cmd = select_recovery_command(
        phase=99,
        reverse_speed=-0.4,
        reverse_steer=0.3,
        rear_blocked=False,
    )
    assert cmd == DriveCommand()


def test_front_deadlock_triggers_when_wall_blocked_and_stopped():
    assert should_trigger_wall_deadlock(**_wall_deadlock_kwargs())


def test_side_deadlock_left_triggers_when_side_is_jammed():
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.35
    kwargs["left_clearance"] = 0.15
    kwargs["right_clearance"] = 0.45
    assert should_trigger_wall_deadlock(**kwargs)


def test_side_deadlock_right_triggers_when_side_is_jammed():
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.35
    kwargs["left_clearance"] = 0.48
    kwargs["right_clearance"] = 0.14
    assert should_trigger_wall_deadlock(**kwargs)


def test_deadlock_triggers_with_unknown_speed_when_command_stopped():
    kwargs = _wall_deadlock_kwargs()
    kwargs["speed_known"] = False
    kwargs["actual_speed"] = 1.0
    assert should_trigger_wall_deadlock(**kwargs)


def test_deadlock_does_not_trigger_with_unknown_speed_if_command_is_forward():
    kwargs = _wall_deadlock_kwargs()
    kwargs["speed_known"] = False
    kwargs["selected_linear_x"] = 0.20
    assert not should_trigger_wall_deadlock(**kwargs)


def test_deadlock_does_not_trigger_in_narrow_symmetric_corridor():
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.50
    kwargs["left_clearance"] = 0.18
    kwargs["right_clearance"] = 0.19
    assert not should_trigger_wall_deadlock(**kwargs)


def test_oblique_stuck_triggers_when_side_very_close_and_speed_zero():
    """Oblique stuck: side wall very close, reactive commanding forward, actual speed zero."""
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.50  # front is clear (wall at 45°)
    kwargs["left_clearance"] = 0.12  # very close side contact
    kwargs["right_clearance"] = 0.45
    kwargs["selected_linear_x"] = 0.50  # reactive still commanding forward
    kwargs["speed_known"] = True
    kwargs["actual_speed"] = 0.02  # but car isn't moving
    assert should_trigger_wall_deadlock(**kwargs)


def test_oblique_stuck_does_not_fire_when_speed_unknown():
    """Oblique stuck must be conservative: require speed_known=True."""
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.50
    kwargs["left_clearance"] = 0.12
    kwargs["right_clearance"] = 0.45
    kwargs["selected_linear_x"] = 0.50
    kwargs["speed_known"] = False
    kwargs["actual_speed"] = 0.0
    assert not should_trigger_wall_deadlock(**kwargs)


def test_oblique_stuck_does_not_fire_in_symmetric_narrow_corridor():
    """Symmetric narrow corridor should NOT trigger oblique stuck (side_asym too low)."""
    kwargs = _wall_deadlock_kwargs()
    kwargs["min_front_dist"] = 0.50
    kwargs["left_clearance"] = 0.14
    kwargs["right_clearance"] = 0.14  # symmetric — side_asym < stuck_side_asymmetry_min
    kwargs["selected_linear_x"] = 0.50
    kwargs["speed_known"] = True
    kwargs["actual_speed"] = 0.02
    assert not should_trigger_wall_deadlock(**kwargs)


def test_recovery_timeout_prioritizes_spin_over_deadlock():
    timeout = select_recovery_timeout(
        stuck_timeout=0.8,
        stuck_imu_timeout_factor=0.4,
        stuck_front_blocked_timeout_factor=0.65,
        stuck_deadlock_unknown_speed_timeout_factor=1.20,
        is_spinning=True,
        is_impact=False,
        wall_deadlock=True,
        speed_known=False,
    )
    assert abs(timeout - (0.8 * 0.4)) < 1e-9
