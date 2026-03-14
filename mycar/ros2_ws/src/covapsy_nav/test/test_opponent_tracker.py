from covapsy_nav.opponent_tracker import OpponentTracker
from covapsy_nav.opponent_tracker import nearest_opponent_ahead
from covapsy_nav.opponent_tracker import predict_passing_opportunity


def test_predict_backwards_compatibility_step():
    tracker = OpponentTracker()
    tracker.update([(1.0, 0.1)], timestamp=0.0)
    tracker.predict(dt=0.1, timestamp=0.1)

    opponents = tracker.get_opponents()
    assert len(opponents) == 1
    assert opponents[0].x > 0.5


def test_module_helpers_find_target_and_decision():
    tracker = OpponentTracker()
    tracker.update([(1.2, -0.2)], timestamp=0.0)
    tracker.update([(1.1, -0.2)], timestamp=0.1)

    target = nearest_opponent_ahead(tracker)
    assert target is not None

    decision = predict_passing_opportunity(
        tracker=tracker,
        car_speed=1.2,
        opponent=target,
    )
    assert decision in {"left", "right", "wait"}


def test_module_helper_returns_wait_without_tracks():
    tracker = OpponentTracker()
    assert predict_passing_opportunity(tracker=tracker, car_speed=1.0) == "wait"
