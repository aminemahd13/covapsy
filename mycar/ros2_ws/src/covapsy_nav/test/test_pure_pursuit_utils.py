import math

import numpy as np

from covapsy_nav.pure_pursuit_utils import find_lookahead_index
from covapsy_nav.pure_pursuit_utils import front_min_distance
from covapsy_nav.pure_pursuit_utils import path_is_looped


def test_front_min_distance_uses_front_sector_only():
    ranges = np.full(360, 4.0, dtype=np.float32)
    ranges[270] = 0.10  # +90deg side obstacle
    ranges[180] = 0.55  # front obstacle
    front_min = front_min_distance(
        ranges_in=ranges,
        angle_min=-math.pi,
        angle_increment=(2.0 * math.pi / 360.0),
        half_angle_deg=20.0,
    )
    assert abs(front_min - 0.55) < 1e-6


def test_path_is_looped_detects_closed_tracks():
    open_path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    looped_path = [(0.0, 0.0), (1.0, 0.2), (0.2, 1.0), (0.1, 0.1)]

    assert path_is_looped(open_path) is False
    assert path_is_looped(looped_path, closure_tol=0.2) is True


def test_find_lookahead_index_progresses_along_non_looped_path():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    idx = find_lookahead_index(path=path, start_idx=0, lookahead_dist=1.6, looped=False)
    assert idx == 2


def test_find_lookahead_index_wraps_on_looped_path():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    idx = find_lookahead_index(path=path, start_idx=3, lookahead_dist=1.3, looped=True)
    assert idx in (0, 1)
