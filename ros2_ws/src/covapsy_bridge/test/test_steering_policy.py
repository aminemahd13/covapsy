from covapsy_bridge.steering_policy import (
    apply_external_steering_mode,
    evaluate_steering_gate,
    steer_rad_to_goal_tick,
)


def test_external_steering_mode_forces_neutral():
    assert apply_external_steering_mode(0.31, True) == 0.0
    assert apply_external_steering_mode(-0.18, False) == -0.18


def test_steer_rad_to_goal_tick_mapping_and_clamp():
    assert steer_rad_to_goal_tick(
        steer_rad=0.0,
        max_steer_rad=0.42,
        center_tick=2048,
        left_tick=2560,
        right_tick=1536,
    ) == 2048
    assert steer_rad_to_goal_tick(
        steer_rad=0.42,
        max_steer_rad=0.42,
        center_tick=2048,
        left_tick=2560,
        right_tick=1536,
    ) == 2560
    assert steer_rad_to_goal_tick(
        steer_rad=-0.42,
        max_steer_rad=0.42,
        center_tick=2048,
        left_tick=2560,
        right_tick=1536,
    ) == 1536
    assert steer_rad_to_goal_tick(
        steer_rad=1.5,
        max_steer_rad=0.42,
        center_tick=2048,
        left_tick=2560,
        right_tick=1536,
    ) == 2560


def test_evaluate_steering_gate_paths():
    run_enable, watchdog, wait_start = evaluate_steering_gate(
        started=False,
        cmd_age_s=0.01,
        watchdog_timeout_s=0.2,
        emergency_brake=False,
    )
    assert run_enable is False
    assert watchdog is False
    assert wait_start is True

    run_enable, watchdog, wait_start = evaluate_steering_gate(
        started=True,
        cmd_age_s=0.5,
        watchdog_timeout_s=0.2,
        emergency_brake=False,
    )
    assert run_enable is False
    assert watchdog is True
    assert wait_start is False

    run_enable, watchdog, wait_start = evaluate_steering_gate(
        started=True,
        cmd_age_s=0.01,
        watchdog_timeout_s=0.2,
        emergency_brake=True,
    )
    assert run_enable is False
    assert watchdog is True
    assert wait_start is False

    run_enable, watchdog, wait_start = evaluate_steering_gate(
        started=True,
        cmd_age_s=0.01,
        watchdog_timeout_s=0.2,
        emergency_brake=False,
    )
    assert run_enable is True
    assert watchdog is False
    assert wait_start is False
