"""ROS-independent helpers for track persistence and saved-track state."""

from __future__ import annotations

import json
from pathlib import Path

from covapsy_nav.track_path_store import resolve_directional_path


def load_points_from_json(path: Path) -> list[tuple[float, float]] | None:
    """Load point list from JSON file supporting both dict and list formats."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None

    points = data.get("points", data if isinstance(data, list) else [])
    if not isinstance(points, list) or not points:
        return None

    out: list[tuple[float, float]] = []
    try:
        for pt in points:
            if isinstance(pt, dict):
                x = float(pt.get("x", 0.0))
                y = float(pt.get("y", 0.0))
            else:
                x = float(pt[0])
                y = float(pt[1])
            out.append((x, y))
    except Exception:
        return None
    return out


def resolve_track_save_path(
    storage_dir: str,
    direction: str,
    red_left_filename: str,
    red_right_filename: str,
    unknown_filename: str,
) -> Path:
    """Resolve output file path used for persistence."""
    return resolve_directional_path(
        storage_dir=storage_dir,
        direction=direction,
        red_left_filename=red_left_filename,
        red_right_filename=red_right_filename,
        unknown_filename=unknown_filename,
    )


def apply_saved_track_loaded(current_state: bool, incoming_msg: bool) -> bool:
    """Stick to True once a saved track is reported as loaded."""
    return bool(current_state) or bool(incoming_msg)
