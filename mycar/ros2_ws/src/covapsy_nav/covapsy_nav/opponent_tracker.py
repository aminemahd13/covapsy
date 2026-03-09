"""Lightweight Kalman-filter opponent tracker — pure Python + numpy.

Tracks one or more opponents detected by LiDAR clustering.
Each track maintains position, velocity, and a confidence score.
Stale tracks are automatically pruned.

Used by tactical_race_node to predict opponent positions for
safe passing decisions.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
_MAX_TRACKS = 6
_ASSOCIATION_GATE_M = 0.60       # max distance to associate detection → track
_INIT_VELOCITY_M_S = 0.0
_PROCESS_NOISE_POS = 0.04        # m² per second
_PROCESS_NOISE_VEL = 0.25        # (m/s)² per second
_MEASUREMENT_NOISE = 0.08        # m²
_PRUNE_AGE_SEC = 1.5             # drop tracks not updated for this long
_MIN_CONFIDENCE_FOR_OUTPUT = 0.20
_CONFIDENCE_GAIN_PER_UPDATE = 0.18
_CONFIDENCE_DECAY_PER_SEC = 0.30
_PREDICTION_HORIZON_SEC = 0.8    # how far ahead to predict for passing decisions


@dataclass
class OpponentTrack:
    """State: [x, y, vx, vy].  Covariance: 4×4."""

    track_id: int
    state: np.ndarray           # shape (4,)
    covariance: np.ndarray      # shape (4, 4)
    confidence: float = 0.3
    last_update: float = 0.0
    age_updates: int = 0

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def vx(self) -> float:
        return float(self.state[2])

    @property
    def vy(self) -> float:
        return float(self.state[3])

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def heading(self) -> float:
        return math.atan2(self.vy, self.vx)

    def predicted_position(self, dt: float) -> Tuple[float, float]:
        """Predict position dt seconds into the future."""
        return (self.x + self.vx * dt, self.y + self.vy * dt)


def _initial_covariance() -> np.ndarray:
    return np.diag([0.15, 0.15, 1.0, 1.0])


def _F_matrix(dt: float) -> np.ndarray:
    """State transition: constant-velocity model."""
    return np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1],
    ], dtype=np.float64)


def _Q_matrix(dt: float) -> np.ndarray:
    """Process noise."""
    q_p = _PROCESS_NOISE_POS * dt
    q_v = _PROCESS_NOISE_VEL * dt
    return np.diag([q_p, q_p, q_v, q_v])


_H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
], dtype=np.float64)

_R = np.eye(2) * _MEASUREMENT_NOISE


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------

class OpponentTracker:
    """Multi-target Kalman tracker for race opponents."""

    def __init__(self, max_tracks: int = _MAX_TRACKS):
        self.tracks: List[OpponentTrack] = []
        self._next_id = 0
        self._max_tracks = max_tracks
        self._last_predict_time: float = 0.0

    def reset(self) -> None:
        self.tracks.clear()
        self._next_id = 0
        self._last_predict_time = 0.0

    # ------------------------------------------------------------------ #
    # Public API                                                          #
    # ------------------------------------------------------------------ #

    def update(
        self,
        detections: List[Tuple[float, float]],
        timestamp: Optional[float] = None,
    ) -> None:
        """Run one predict-update cycle with new detections [(x, y), ...]."""
        now = timestamp if timestamp is not None else time.monotonic()
        dt = now - self._last_predict_time if self._last_predict_time > 0.0 else 0.02
        dt = max(0.001, min(dt, 1.0))
        self._last_predict_time = now

        # 1) Predict all existing tracks
        self._predict_all(dt, now)

        # 2) Associate detections to tracks (greedy nearest-neighbour)
        used_detections = set()
        for track in self.tracks:
            best_idx = -1
            best_dist = _ASSOCIATION_GATE_M
            for i, (dx, dy) in enumerate(detections):
                if i in used_detections:
                    continue
                dist = math.hypot(dx - track.x, dy - track.y)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx >= 0:
                self._kf_update(track, detections[best_idx], now)
                used_detections.add(best_idx)

        # 3) Spawn new tracks for unassociated detections
        for i, (dx, dy) in enumerate(detections):
            if i not in used_detections and len(self.tracks) < self._max_tracks:
                self._spawn(dx, dy, now)

        # 4) Prune stale tracks
        self._prune(now)

    def get_opponents(self) -> List[OpponentTrack]:
        """Return currently active tracks above confidence threshold."""
        return [
            t for t in self.tracks
            if t.confidence >= _MIN_CONFIDENCE_FOR_OUTPUT
        ]

    def nearest_opponent_ahead(
        self,
        car_x: float = 0.0,
        car_y: float = 0.0,
        car_yaw: float = 0.0,
        half_angle_rad: float = math.radians(45),
    ) -> Optional[OpponentTrack]:
        """Return the nearest opponent in the forward cone, or None."""
        best: Optional[OpponentTrack] = None
        best_dist = float("inf")

        for t in self.get_opponents():
            dx = t.x - car_x
            dy = t.y - car_y
            # Transform to car frame
            local_x = dx * math.cos(-car_yaw) - dy * math.sin(-car_yaw)
            local_y = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
            if local_x <= 0:
                continue
            angle = abs(math.atan2(local_y, local_x))
            if angle > half_angle_rad:
                continue
            dist = math.hypot(local_x, local_y)
            if dist < best_dist:
                best_dist = dist
                best = t

        return best

    def predict_passing_opportunity(
        self,
        car_speed: float,
        opponent: OpponentTrack,
        horizon_sec: float = _PREDICTION_HORIZON_SEC,
    ) -> str:
        """Return 'left', 'right', or 'wait' based on predicted trajectories."""
        if opponent.speed < 0.05:
            # Opponent is stationary — check which side has more room
            return "left" if opponent.y > 0 else "right"

        # Predict where opponent will be
        pred_x, pred_y = opponent.predicted_position(horizon_sec)

        # If opponent is moving slower, we can pass; bias to clearer side
        if car_speed > opponent.speed * 1.15:
            return "left" if pred_y >= 0 else "right"

        return "wait"

    # ------------------------------------------------------------------ #
    # Internal                                                            #
    # ------------------------------------------------------------------ #

    def _predict_all(self, dt: float, now: float) -> None:
        F = _F_matrix(dt)
        Q = _Q_matrix(dt)
        for track in self.tracks:
            track.state = F @ track.state
            track.covariance = F @ track.covariance @ F.T + Q
            elapsed = now - track.last_update
            track.confidence = max(0.0, track.confidence - _CONFIDENCE_DECAY_PER_SEC * elapsed)

    def _kf_update(
        self,
        track: OpponentTrack,
        measurement: Tuple[float, float],
        now: float,
    ) -> None:
        z = np.array(measurement, dtype=np.float64)
        y = z - _H @ track.state
        S = _H @ track.covariance @ _H.T + _R
        try:
            K = track.covariance @ _H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        track.state = track.state + K @ y
        I4 = np.eye(4)
        track.covariance = (I4 - K @ _H) @ track.covariance
        track.last_update = now
        track.age_updates += 1
        track.confidence = min(1.0, track.confidence + _CONFIDENCE_GAIN_PER_UPDATE)

    def _spawn(self, x: float, y: float, now: float) -> None:
        state = np.array([x, y, _INIT_VELOCITY_M_S, _INIT_VELOCITY_M_S], dtype=np.float64)
        track = OpponentTrack(
            track_id=self._next_id,
            state=state,
            covariance=_initial_covariance(),
            confidence=0.25,
            last_update=now,
            age_updates=1,
        )
        self.tracks.append(track)
        self._next_id += 1

    def _prune(self, now: float) -> None:
        self.tracks = [
            t for t in self.tracks
            if (now - t.last_update) < _PRUNE_AGE_SEC or t.confidence > 0.05
        ]


# ---------------------------------------------------------------------------
# LiDAR cluster extraction for feeding the tracker
# ---------------------------------------------------------------------------

def extract_opponent_clusters(
    ranges: List[float],
    angles: List[float],
    car_width: float = 0.30,
    min_range: float = 0.30,
    max_range: float = 3.5,
    min_cluster_points: int = 3,
    max_cluster_width_m: float = 0.55,
) -> List[Tuple[float, float]]:
    """Extract opponent centroids from LiDAR scan as (x, y) in car frame.

    Returns list of (x, y) tuples for detected obstacles that likely are cars.
    Filters out walls (too wide) and noise (too few points).
    """
    # Convert polar to cartesian and find clusters
    points: List[Tuple[float, float, int]] = []
    for i, (r, a) in enumerate(zip(ranges, angles)):
        if not math.isfinite(r) or r < min_range or r > max_range:
            continue
        x = r * math.cos(a)
        y = r * math.sin(a)
        points.append((x, y, i))

    if len(points) < min_cluster_points:
        return []

    # Simple sequential clustering by angular adjacency
    clusters: List[List[Tuple[float, float]]] = []
    current_cluster: List[Tuple[float, float]] = [(points[0][0], points[0][1])]

    for k in range(1, len(points)):
        prev_idx = points[k - 1][2]
        curr_idx = points[k][2]
        dx = points[k][0] - points[k - 1][0]
        dy = points[k][1] - points[k - 1][1]
        gap = math.hypot(dx, dy)

        if gap < car_width * 1.5 and abs(curr_idx - prev_idx) < 8:
            current_cluster.append((points[k][0], points[k][1]))
        else:
            if len(current_cluster) >= min_cluster_points:
                clusters.append(current_cluster)
            current_cluster = [(points[k][0], points[k][1])]

    if len(current_cluster) >= min_cluster_points:
        clusters.append(current_cluster)

    # Filter and compute centroids
    centroids: List[Tuple[float, float]] = []
    for cluster in clusters:
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        extent = max(width, height)

        # Filter: opponent cars are ~0.30-0.48m wide, walls are much wider
        if extent > max_cluster_width_m:
            continue
        if extent < 0.08:
            continue

        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        centroids.append((cx, cy))

    return centroids
