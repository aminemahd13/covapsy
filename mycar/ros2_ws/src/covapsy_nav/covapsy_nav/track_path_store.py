"""Helpers for direction-aware racing path storage."""

from __future__ import annotations

from pathlib import Path


_VALID_DIRECTIONS = ("unknown", "red_left", "red_right")


def normalize_track_direction(direction: str) -> str:
    """Normalize track direction labels to known values."""
    value = str(direction).strip().lower()
    if value not in _VALID_DIRECTIONS:
        return "unknown"
    return value


def resolve_directional_path(
    storage_dir: str,
    direction: str,
    red_left_filename: str,
    red_right_filename: str,
    unknown_filename: str,
) -> Path:
    """Resolve persisted path file from storage root and direction."""
    root = Path(storage_dir).expanduser()
    norm_dir = normalize_track_direction(direction)
    if norm_dir == "red_left":
        filename = red_left_filename
    elif norm_dir == "red_right":
        filename = red_right_filename
    else:
        filename = unknown_filename
    return root / str(filename).strip()
