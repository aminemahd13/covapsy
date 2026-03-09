"""Track learning and racing line optimisation — pure Python + numpy.

During the 3 permitted setup laps (SLAM/exploration):
  1. Record (x, y) positions from odometry
  2. Detect when a full lap is complete (closure detection)
  3. Smooth the recorded path
  4. Optimise the racing line (apex cutting, exit widening)
  5. Export as JSON waypoints for pure-pursuit

The racing line optimiser uses iterative apex-cutting:
  - Shift each waypoint toward the inside of curves
  - Smooth the result with a moving average
  - Recompute curvature and assign speed targets per waypoint
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
_CLOSURE_TOL_M = 0.50        # how close last point must be to first for lap detection
_MIN_LAP_POINTS = 40         # minimum waypoints for a valid lap
_MIN_SPACING_M = 0.06        # minimum distance between recorded points
_SMOOTH_WINDOW = 7           # moving-average window for path smoothing
_APEX_ITERATIONS = 5         # number of apex-cutting passes
_APEX_ALPHA = 0.12           # how much to shift toward inside per iteration
_APEX_BETA = 0.08            # smoothing factor per iteration
_SPEED_CURV_BASE = 2.2       # speed at zero curvature (m/s)
_SPEED_CURV_GAIN = 4.0       # curvature→speed penalty
_SPEED_MIN = 0.45            # minimum assigned speed
_SPEED_MAX = 2.8             # maximum assigned speed
_TRACK_WIDTH_ESTIMATE = 0.80 # approximate track half-width for apex cutting


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Track recording
# ---------------------------------------------------------------------------

class TrackLearner:
    """Accumulates odometry positions and learns the track shape."""

    def __init__(
        self,
        min_spacing: float = _MIN_SPACING_M,
        closure_tol: float = _CLOSURE_TOL_M,
    ):
        self.raw_points: List[Tuple[float, float]] = []
        self.laps_detected: int = 0
        self.lap_points: Optional[List[Tuple[float, float]]] = None
        self.racing_line: Optional[List[Dict]] = None
        self._min_spacing = min_spacing
        self._closure_tol = closure_tol
        self._last_x = float("nan")
        self._last_y = float("nan")

    def add_position(self, x: float, y: float) -> bool:
        """Record an odometry position.  Returns True if a lap was just completed."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return False

        # Enforce minimum spacing
        if self.raw_points:
            dx = x - self._last_x
            dy = y - self._last_y
            if math.hypot(dx, dy) < self._min_spacing:
                return False

        self.raw_points.append((x, y))
        self._last_x = x
        self._last_y = y

        # Closure detection — need enough points and close to start
        if len(self.raw_points) >= _MIN_LAP_POINTS:
            sx, sy = self.raw_points[0]
            if math.hypot(x - sx, y - sy) < self._closure_tol:
                self.laps_detected += 1
                self.lap_points = list(self.raw_points)
                self.raw_points = [(x, y)]  # start new lap recording
                return True

        return False

    @property
    def has_lap(self) -> bool:
        return self.lap_points is not None and len(self.lap_points) >= _MIN_LAP_POINTS

    def build_racing_line(self) -> Optional[List[Dict]]:
        """Process recorded lap into an optimised racing line.

        Returns list of dicts: [{x, y, speed, curvature}, ...] or None.
        """
        if not self.has_lap:
            return None

        pts = np.array(self.lap_points, dtype=np.float64)

        # 1) Smooth
        pts = _smooth_path(pts, window=_SMOOTH_WINDOW)

        # 2) Resample at even spacing
        pts = _resample_path(pts, spacing=max(self._min_spacing, 0.08))

        if len(pts) < 10:
            return None

        # 3) Compute curvature at each point
        curvatures = _compute_curvatures(pts)

        # 4) Apex cutting — iteratively shift toward inside of curves
        pts = _apex_cut(pts, curvatures, iterations=_APEX_ITERATIONS)

        # 5) Re-smooth after apex cutting
        pts = _smooth_path(pts, window=max(3, _SMOOTH_WINDOW // 2))

        # 6) Recompute curvature on optimised line
        curvatures = _compute_curvatures(pts)

        # 7) Assign speed targets
        speeds = _assign_speeds(curvatures)

        # 8) Build output
        self.racing_line = []
        for i in range(len(pts)):
            self.racing_line.append({
                "x": float(pts[i, 0]),
                "y": float(pts[i, 1]),
                "speed": float(speeds[i]),
                "curvature": float(curvatures[i]),
            })

        return self.racing_line

    def export_json(self, filepath: str) -> None:
        """Export racing line to JSON file for racing_path_publisher."""
        if self.racing_line is None:
            self.build_racing_line()
        if self.racing_line is None:
            return
        data = {
            "points": [{"x": p["x"], "y": p["y"]} for p in self.racing_line],
            "speeds": [p["speed"] for p in self.racing_line],
            "curvatures": [p["curvature"] for p in self.racing_line],
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def get_path_xy(self) -> Optional[List[Tuple[float, float]]]:
        """Return optimised path as [(x,y), ...] for pure pursuit."""
        if self.racing_line is None:
            return None
        return [(p["x"], p["y"]) for p in self.racing_line]


# ---------------------------------------------------------------------------
# Path processing helpers
# ---------------------------------------------------------------------------

def _smooth_path(pts: np.ndarray, window: int = 7) -> np.ndarray:
    """Circular moving-average smoothing."""
    n = len(pts)
    if n < window:
        return pts
    half = window // 2
    smoothed = np.empty_like(pts)
    for i in range(n):
        indices = [(i + j) % n for j in range(-half, half + 1)]
        smoothed[i] = np.mean(pts[indices], axis=0)
    return smoothed


def _resample_path(pts: np.ndarray, spacing: float = 0.08) -> np.ndarray:
    """Resample path at approximately even arc-length spacing."""
    if len(pts) < 2:
        return pts

    # Compute cumulative arc length
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_lengths = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_length = cum_lengths[-1]

    if total_length < spacing:
        return pts

    num_points = max(10, int(total_length / spacing))
    target_lengths = np.linspace(0, total_length, num_points, endpoint=False)

    resampled = np.empty((num_points, 2))
    for i, target in enumerate(target_lengths):
        idx = np.searchsorted(cum_lengths, target, side="right") - 1
        idx = max(0, min(idx, len(pts) - 2))
        seg_len = seg_lengths[idx]
        if seg_len < 1e-6:
            resampled[i] = pts[idx]
        else:
            t = (target - cum_lengths[idx]) / seg_len
            resampled[i] = pts[idx] * (1 - t) + pts[idx + 1] * t

    return resampled


def _compute_curvatures(pts: np.ndarray) -> np.ndarray:
    """Compute discrete curvature at each point (circular)."""
    n = len(pts)
    curvatures = np.zeros(n)

    for i in range(n):
        p0 = pts[(i - 1) % n]
        p1 = pts[i]
        p2 = pts[(i + 1) % n]

        d1 = p1 - p0
        d2 = p2 - p1

        cross = abs(d1[0] * d2[1] - d1[1] * d2[0])
        l1 = np.linalg.norm(d1)
        l2 = np.linalg.norm(d2)
        l3 = np.linalg.norm(p2 - p0)

        denom = l1 * l2 * l3
        if denom > 1e-9:
            curvatures[i] = 2.0 * cross / denom
        else:
            curvatures[i] = 0.0

    return curvatures


def _apex_cut(
    pts: np.ndarray,
    curvatures: np.ndarray,
    iterations: int = _APEX_ITERATIONS,
) -> np.ndarray:
    """Iteratively cut apexes to create a racing line.

    Shift high-curvature points toward the inside of the curve,
    then smooth.  This mimics what racing drivers do naturally.
    """
    result = pts.copy()
    n = len(result)

    for _iteration in range(iterations):
        new_pts = result.copy()
        for i in range(n):
            if curvatures[i] < 0.5:
                continue  # don't move points on straights

            p_prev = result[(i - 1) % n]
            p_next = result[(i + 1) % n]
            midpoint = 0.5 * (p_prev + p_next)

            # Direction toward inside of curve (toward midpoint of neighbours)
            shift = midpoint - result[i]
            shift_mag = np.linalg.norm(shift)

            if shift_mag < 1e-6:
                continue

            # Scale shift by curvature (more curvature = more apex cutting)
            curv_factor = _clamp(curvatures[i] / 5.0, 0.0, 1.0)
            shift_amount = _APEX_ALPHA * curv_factor * _TRACK_WIDTH_ESTIMATE

            new_pts[i] = result[i] + (shift / shift_mag) * shift_amount

        # Smooth between iterations
        result = _smooth_path(new_pts, window=3)

    return result


def _assign_speeds(curvatures: np.ndarray) -> np.ndarray:
    """Assign target speed at each waypoint based on curvature.

    Uses a simple model: speed = base - gain × curvature, clamped.
    Then applies forward/backward propagation for smooth braking/accel.
    """
    n = len(curvatures)
    speeds = np.empty(n)

    for i in range(n):
        s = _SPEED_CURV_BASE - _SPEED_CURV_GAIN * curvatures[i]
        speeds[i] = _clamp(s, _SPEED_MIN, _SPEED_MAX)

    # Forward propagation: can't accelerate infinitely fast
    max_accel = 1.5  # m/s per waypoint (heuristic)
    for i in range(1, n):
        speeds[i] = min(speeds[i], speeds[i - 1] + max_accel * 0.08)

    # Backward propagation: must brake before slow sections
    max_decel = 2.0  # m/s per waypoint
    for i in range(n - 2, -1, -1):
        speeds[i] = min(speeds[i], speeds[(i + 1) % n] + max_decel * 0.08)

    return speeds
