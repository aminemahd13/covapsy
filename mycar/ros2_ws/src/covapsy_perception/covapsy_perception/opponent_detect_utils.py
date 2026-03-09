"""Utility functions for lightweight opponent detection."""

from __future__ import annotations

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def lidar_opponent_confidence(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    detect_range: float = 2.8,
) -> tuple[float, float, float, float]:
    """Return (confidence, front_min, left_clear, right_clear)."""
    values = np.array(ranges, dtype=np.float32)
    if values.size == 0:
        return 0.0, 5.0, 5.0, 5.0

    values = np.where(np.isfinite(values), values, detect_range)
    values = np.clip(values, 0.05, max(detect_range, 0.1))
    angles = float(angle_min) + np.arange(values.size, dtype=np.float32) * float(angle_increment)

    front_mask = np.abs(angles) < np.radians(18.0)
    left_mask = (angles > np.radians(20.0)) & (angles < np.radians(75.0))
    right_mask = (angles < -np.radians(20.0)) & (angles > -np.radians(75.0))

    front = values[front_mask]
    left = values[left_mask]
    right = values[right_mask]
    front_min = float(np.min(front)) if front.size > 0 else detect_range
    left_clear = float(np.median(left)) if left.size > 0 else detect_range
    right_clear = float(np.median(right)) if right.size > 0 else detect_range

    near_front = values[(np.abs(angles) < np.radians(65.0)) & (values < detect_range)]
    points_score = min(1.0, near_front.size / 30.0)
    dist_score = clamp((detect_range - front_min) / max(detect_range, 1e-3), 0.0, 1.0)
    conf = clamp(0.55 * points_score + 0.45 * dist_score, 0.0, 1.0)
    return conf, front_min, left_clear, right_clear


def fused_confidence(lidar_conf: float, camera_conf: float, camera_weight: float = 0.2) -> float:
    lw = 1.0 - clamp(camera_weight, 0.0, 0.8)
    cw = 1.0 - lw
    return clamp((lw * lidar_conf) + (cw * camera_conf), 0.0, 1.0)
