"""Utilities for LiDAR scan preprocessing."""

from __future__ import annotations

import numpy as np


def filter_scan_ranges(
    ranges: np.ndarray,
    max_range: float,
    min_range: float,
    median_window: int,
) -> np.ndarray:
    """Return filtered scan ranges with clipping and optional median smoothing."""
    values = np.array(ranges, dtype=np.float32)
    values = np.where(np.isfinite(values), values, max_range)
    values = np.clip(values, min_range, max_range)

    if median_window > 1:
        pad = median_window // 2
        padded = np.pad(values, pad, mode="edge")
        shape = (len(values), median_window)
        strides = (padded.strides[0], padded.strides[0])
        windows = np.lib.stride_tricks.as_strided(padded, shape=shape, strides=strides)
        values = np.median(windows, axis=1).astype(np.float32)
    return values

