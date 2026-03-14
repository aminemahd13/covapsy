from covapsy_perception.border_detect_utils import update_wrong_way_hysteresis


def test_wrong_way_hysteresis_enters_after_confirm_frames():
    active = False
    count = 0
    for _ in range(6):
        active, count = update_wrong_way_hysteresis(
            wrong_conf=0.8,
            active=active,
            confirm_count=count,
            enter_conf=0.55,
            exit_conf=0.35,
            confirm_frames=6,
        )
    assert active is True
    assert count == 6


def test_wrong_way_hysteresis_does_not_toggle_on_single_spike():
    active = False
    count = 0
    active, count = update_wrong_way_hysteresis(
        wrong_conf=0.7,
        active=active,
        confirm_count=count,
        enter_conf=0.55,
        exit_conf=0.35,
        confirm_frames=4,
    )
    active, count = update_wrong_way_hysteresis(
        wrong_conf=0.1,
        active=active,
        confirm_count=count,
        enter_conf=0.55,
        exit_conf=0.35,
        confirm_frames=4,
    )
    assert active is False
    assert count == 0


def test_wrong_way_hysteresis_exits_only_after_decay():
    active = True
    count = 4
    for _ in range(3):
        active, count = update_wrong_way_hysteresis(
            wrong_conf=0.2,
            active=active,
            confirm_count=count,
            enter_conf=0.55,
            exit_conf=0.35,
            confirm_frames=4,
        )
    assert active is True
    assert count == 1
    active, count = update_wrong_way_hysteresis(
        wrong_conf=0.2,
        active=active,
        confirm_count=count,
        enter_conf=0.55,
        exit_conf=0.35,
        confirm_frames=4,
    )
    assert active is False
    assert count == 0
