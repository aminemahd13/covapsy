import math

from covapsy_nav.track_learner import TrackLearner


def _circle_points(radius: float, count: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(count):
        angle = (2.0 * math.pi * i) / count
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def test_build_racing_line_with_custom_smoothing_and_apex_iterations():
    learner = TrackLearner(
        min_spacing=0.06,
        closure_tol=0.50,
        smoothing_window=9,
        apex_iterations=2,
    )
    learner.lap_points = _circle_points(radius=2.0, count=120)

    line = learner.build_racing_line()
    assert line is not None
    assert len(line) >= 20
    assert all("x" in p and "y" in p and "speed" in p and "curvature" in p for p in line)


def test_smoothing_window_is_safely_clamped():
    learner = TrackLearner(
        min_spacing=0.06,
        closure_tol=0.50,
        smoothing_window=1,
        apex_iterations=1,
    )
    learner.lap_points = _circle_points(radius=1.5, count=100)

    line = learner.build_racing_line()
    assert line is not None
    assert len(line) >= 20
