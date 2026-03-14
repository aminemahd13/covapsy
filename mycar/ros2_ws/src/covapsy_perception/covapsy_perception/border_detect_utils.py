"""Pure helpers for border-based wrong-way detection."""

from __future__ import annotations


_VALID_TRACK_DIRECTIONS = ("unknown", "red_left", "red_right")


def normalize_track_direction(direction: str) -> str:
    """Normalize and validate track direction labels."""
    value = str(direction).strip().lower()
    if value not in _VALID_TRACK_DIRECTIONS:
        return "unknown"
    return value


def update_track_direction_hysteresis(
    observed_direction: str,
    active_direction: str,
    candidate_direction: str,
    candidate_count: int,
    confirm_frames: int,
) -> tuple[str, str, int]:
    """Update direction state with short confirmation to prevent flicker."""
    observed = normalize_track_direction(observed_direction)
    active = normalize_track_direction(active_direction)
    candidate = normalize_track_direction(candidate_direction)
    frames = max(1, int(confirm_frames))
    count = max(0, int(candidate_count))

    if observed == "unknown":
        return active, "unknown", 0

    if observed == active:
        return active, observed, frames

    if observed != candidate:
        candidate = observed
        count = 1
    else:
        count = min(frames, count + 1)

    if count >= frames:
        active = candidate
        return active, candidate, frames

    return active, candidate, count


def update_wrong_way_hysteresis(
    wrong_conf: float,
    active: bool,
    confirm_count: int,
    enter_conf: float,
    exit_conf: float,
    confirm_frames: int,
) -> tuple[bool, int]:
    """Update wrong-way hysteresis state from confidence input."""
    frames = max(1, int(confirm_frames))
    count = int(confirm_count)
    conf = float(wrong_conf)
    if conf >= float(enter_conf):
        count = min(frames, count + 1)
    elif conf <= float(exit_conf):
        count = max(0, count - 1)

    state = bool(active)
    if not state and count >= frames:
        state = True
    elif state and conf <= float(exit_conf) and count == 0:
        state = False
    return state, count
