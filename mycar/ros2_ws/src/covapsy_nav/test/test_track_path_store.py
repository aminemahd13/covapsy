from covapsy_nav.track_path_store import normalize_track_direction
from covapsy_nav.track_path_store import resolve_directional_path


def test_normalize_track_direction_defaults_to_unknown():
    assert normalize_track_direction("RED_LEFT") == "red_left"
    assert normalize_track_direction("red_right") == "red_right"
    assert normalize_track_direction("invalid") == "unknown"


def test_resolve_directional_path_uses_expected_filename():
    storage = "/tmp/covapsy_store"
    red_left = "left.json"
    red_right = "right.json"
    unknown = "unknown.json"
    assert str(
        resolve_directional_path(storage, "red_left", red_left, red_right, unknown)
    ).endswith("/tmp/covapsy_store/left.json")
    assert str(
        resolve_directional_path(storage, "red_right", red_left, red_right, unknown)
    ).endswith("/tmp/covapsy_store/right.json")
    assert str(
        resolve_directional_path(storage, "unknown", red_left, red_right, unknown)
    ).endswith("/tmp/covapsy_store/unknown.json")
