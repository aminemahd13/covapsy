import numpy as np

from covapsy_perception.opponent_detect_utils import fused_confidence
from covapsy_perception.opponent_detect_utils import lidar_opponent_confidence


def test_lidar_opponent_confidence_detects_front_cluster():
    ranges = np.full(360, 5.0, dtype=np.float32)
    ranges[170:190] = 0.8
    conf, front_min, _left, _right = lidar_opponent_confidence(
        ranges=ranges,
        angle_min=-np.pi,
        angle_increment=(2.0 * np.pi) / 360.0,
        detect_range=2.8,
    )
    assert conf > 0.2
    assert front_min < 1.0


def test_lidar_opponent_confidence_low_on_clear_track():
    ranges = np.full(360, 5.0, dtype=np.float32)
    conf, front_min, _left, _right = lidar_opponent_confidence(
        ranges=ranges,
        angle_min=-np.pi,
        angle_increment=(2.0 * np.pi) / 360.0,
        detect_range=2.8,
    )
    assert conf < 0.2
    assert front_min >= 2.7


def test_fused_confidence_blends_camera_and_lidar():
    conf = fused_confidence(lidar_conf=0.8, camera_conf=0.2, camera_weight=0.25)
    assert 0.5 < conf < 0.8
