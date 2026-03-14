from covapsy_nav.track_persistence_utils import apply_saved_track_loaded
from covapsy_nav.track_persistence_utils import resolve_track_save_path


def test_apply_saved_track_loaded_is_sticky_true():
    state = apply_saved_track_loaded(False, False)
    assert state is False
    state = apply_saved_track_loaded(state, True)
    assert state is True
    state = apply_saved_track_loaded(state, False)
    assert state is True


def test_resolve_track_save_path_uses_unknown_fallback():
    path = resolve_track_save_path(
        storage_dir="/tmp/covapsy_store",
        direction="unknown",
        red_left_filename="left.json",
        red_right_filename="right.json",
        unknown_filename="unknown.json",
    )
    assert str(path).endswith("/tmp/covapsy_store/unknown.json")
