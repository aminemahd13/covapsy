import json

from covapsy_nav.track_persistence_utils import load_points_from_json


def test_load_points_supports_dict_points(tmp_path):
    path = tmp_path / "path.json"
    path.write_text(
        json.dumps({"points": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]}),
        encoding="utf-8",
    )
    assert load_points_from_json(path) == [(1.0, 2.0), (3.0, 4.0)]


def test_load_points_returns_none_on_missing_points(tmp_path):
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps({"foo": "bar"}), encoding="utf-8")
    assert load_points_from_json(path) is None
