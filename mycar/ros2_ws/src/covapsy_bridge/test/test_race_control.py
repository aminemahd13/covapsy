from covapsy_bridge.race_control import RaceControlPolicy
from covapsy_bridge.race_control import RaceControlState
from covapsy_bridge.race_control import apply_race_signal
from covapsy_bridge.race_control import race_command_allowed


def test_competition_mode_requires_start_signal():
    policy = RaceControlPolicy(
        competition_mode=True,
        require_start_signal=True,
        allow_restart_after_stop=False,
    )
    state = RaceControlState(race_running=False, stop_latched=False)

    assert race_command_allowed(policy, state) is False

    started = apply_race_signal(state, start=True, stop=False, policy=policy)
    assert started.race_running is True
    assert started.stop_latched is False
    assert race_command_allowed(policy, started) is True


def test_stop_signal_latches_and_blocks_restart_by_default():
    policy = RaceControlPolicy(
        competition_mode=True,
        require_start_signal=True,
        allow_restart_after_stop=False,
    )
    running = RaceControlState(race_running=True, stop_latched=False)

    stopped = apply_race_signal(running, start=False, stop=True, policy=policy)
    assert stopped.race_running is False
    assert stopped.stop_latched is True
    assert race_command_allowed(policy, stopped) is False

    restarted = apply_race_signal(stopped, start=True, stop=False, policy=policy)
    assert restarted == stopped


def test_restart_allowed_when_enabled():
    policy = RaceControlPolicy(
        competition_mode=True,
        require_start_signal=True,
        allow_restart_after_stop=True,
    )
    stopped = RaceControlState(race_running=False, stop_latched=True)

    restarted = apply_race_signal(stopped, start=True, stop=False, policy=policy)
    assert restarted.race_running is True
    assert restarted.stop_latched is False
    assert race_command_allowed(policy, restarted) is True


def test_non_competition_mode_does_not_gate_drive_commands():
    policy = RaceControlPolicy(
        competition_mode=False,
        require_start_signal=True,
        allow_restart_after_stop=False,
    )
    state = RaceControlState(race_running=False, stop_latched=False)
    assert race_command_allowed(policy, state) is True

    stopped = apply_race_signal(state, start=False, stop=True, policy=policy)
    assert stopped.stop_latched is False
    assert race_command_allowed(policy, stopped) is True

