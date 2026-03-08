import numpy as np

from covapsy_perception.scan_filter_utils import filter_scan_ranges


def test_filter_scan_ranges_clips_and_replaces_invalid():
    raw = np.array([np.nan, np.inf, 0.0, 7.0, 1.0], dtype=np.float32)
    out = filter_scan_ranges(raw, max_range=5.0, min_range=0.15, median_window=1)
    assert np.all(np.isfinite(out))
    assert np.all(out >= 0.15)
    assert np.all(out <= 5.0)
    assert out[0] == 5.0
    assert out[2] == 0.15


def test_filter_scan_ranges_median_smooths_spikes():
    raw = np.array([1.0, 1.0, 5.0, 1.0, 1.0], dtype=np.float32)
    out = filter_scan_ranges(raw, max_range=5.0, min_range=0.1, median_window=3)
    assert float(out[2]) == 1.0

