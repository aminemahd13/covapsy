"""Race start/stop gating helpers for rules-compliant command handling."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RaceControlPolicy:
    """Rules-aligned gate for when external drive commands are accepted."""

    competition_mode: bool = True
    require_start_signal: bool = True
    allow_restart_after_stop: bool = False


@dataclass(frozen=True)
class RaceControlState:
    """Current race control state driven by /race_start and /race_stop."""

    race_running: bool = False
    stop_latched: bool = False


def apply_race_signal(
    current: RaceControlState,
    *,
    start: bool,
    stop: bool,
    policy: RaceControlPolicy,
) -> RaceControlState:
    """Apply race start/stop signals and return updated state."""
    if stop:
        return RaceControlState(race_running=False, stop_latched=policy.competition_mode)

    if not start:
        return current

    if policy.competition_mode and current.stop_latched and not policy.allow_restart_after_stop:
        return current

    return RaceControlState(race_running=True, stop_latched=False)


def race_command_allowed(policy: RaceControlPolicy, state: RaceControlState) -> bool:
    """Return True when drive input should be acted on."""
    if not policy.competition_mode:
        return True
    if state.stop_latched:
        return False
    if policy.require_start_signal and not state.race_running:
        return False
    return True

