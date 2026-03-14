import math

import numpy as np

from covapsy_nav.pure_pursuit_utils import choose_progress_index
from covapsy_nav.pure_pursuit_utils import find_forward_lookahead_index
from covapsy_nav.pure_pursuit_utils import find_lookahead_index
from covapsy_nav.pure_pursuit_utils import front_min_distance
from covapsy_nav.pure_pursuit_utils import path_is_looped
from covapsy_nav.pure_pursuit_utils import to_vehicle_frame


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


def test_choose_progress_index_blocks_small_backward_jitter_non_looped():
    idx = choose_progress_index(
        nearest_idx=7,
        prev_progress_idx=10,
        path_length=20,
        looped=False,
        nearest_dist=0.2,
        relocalization_distance_m=0.9,
        max_backward_index_jump=4,
    )
    assert idx == 10


def test_choose_progress_index_allows_relocalization_jump_non_looped():
    idx = choose_progress_index(
        nearest_idx=7,
        prev_progress_idx=10,
        path_length=20,
        looped=False,
        nearest_dist=1.4,
        relocalization_distance_m=0.9,
        max_backward_index_jump=4,
    )
    assert idx == 7


def test_choose_progress_index_allows_loop_wrap_but_blocks_small_backstep():
    n = 100
    # Legitimate wrap near loop boundary (forward progress through index 0)
    wrap_idx = choose_progress_index(
        nearest_idx=2,
        prev_progress_idx=98,
        path_length=n,
        looped=True,
        nearest_dist=0.2,
        relocalization_distance_m=0.9,
        max_backward_index_jump=6,
    )
    assert wrap_idx == 2
    # Small backward jitter should be rejected.
    jitter_idx = choose_progress_index(
        nearest_idx=18,
        prev_progress_idx=20,
        path_length=n,
        looped=True,
        nearest_dist=0.2,
        relocalization_distance_m=0.9,
        max_backward_index_jump=6,
    )
    assert jitter_idx == 20


def test_find_forward_lookahead_index_skips_behind_targets():
    # Car at x=2 facing +x, so waypoints at x<2 are behind.
    path = [(0.0, 0.0), (1.0, 0.0), (2.5, 0.0), (3.0, 0.0)]
    idx = find_forward_lookahead_index(
        path=path,
        start_idx=0,
        lookahead_dist=0.2,
        looped=False,
        pose_x=2.0,
        pose_y=0.0,
        yaw_rad=0.0,
        min_forward_x_m=0.05,
    )
    assert idx in (2, 3)


def test_to_vehicle_frame_basic_transform():
    local_x, local_y = to_vehicle_frame(
        target_x=2.0,
        target_y=1.0,
        pose_x=1.0,
        pose_y=1.0,
        yaw_rad=0.0,
    )
    assert abs(local_x - 1.0) < 1e-6
    assert abs(local_y) < 1e-6
