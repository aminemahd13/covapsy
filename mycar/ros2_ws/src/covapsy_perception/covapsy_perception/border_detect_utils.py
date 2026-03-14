"""Pure helpers for border-based wrong-way detection."""

from __future__ import annotations


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
