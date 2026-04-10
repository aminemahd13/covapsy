from covapsy_bridge.lcd_status_formatter import (
    RECOVERY_BRAKE,
    LcdStatusSnapshot,
    build_lcd_lines,
)


def test_bridge_alert_has_top_priority():
    snapshot = LcdStatusSnapshot(
        bridge_status='USB_TIMEOUT',
        steering_status='OK',
        car_mode='RACE',
        recovery_state=RECOVERY_BRAKE,
    )
    lines = build_lcd_lines(snapshot, page_index=1)
    assert lines[0] == 'ALERT BRIDGE'
    assert 'USB_TOUT' in lines[1]


def test_steering_alert_priority_when_bridge_ok():
    snapshot = LcdStatusSnapshot(
        bridge_status='RUN',
        steering_status='DXL_NO_RESPONSE',
        car_mode='RACE',
    )
    lines = build_lcd_lines(snapshot, page_index=0)
    assert lines[0] == 'ALERT STEER'
    assert 'DXL_NRESP' in lines[1]


def test_recovery_alert_when_active():
    snapshot = LcdStatusSnapshot(
        bridge_status='RUN',
        steering_status='OK',
        car_mode='RECOVERY',
        recovery_state=RECOVERY_BRAKE,
        recovery_attempt=2,
        recovery_reason='front_blocked',
    )
    lines = build_lcd_lines(snapshot, page_index=0)
    assert lines[0] == 'ALERT RECOVERY'
    assert lines[2] == 'FRONT_BLOCKED'


def test_normal_pages_rotate():
    snapshot = LcdStatusSnapshot(
        bridge_status='RUN',
        steering_status='OK',
        car_mode='RACE',
        cmd_speed_mps=0.6,
        wheel_speed_mps=0.5,
        front_clearance_m=0.42,
        track_quality_score=0.81,
    )
    page_a = build_lcd_lines(snapshot, page_index=0)
    page_b = build_lcd_lines(snapshot, page_index=1)

    assert page_a[0] == 'MODE RACE'
    assert page_b[0].startswith('CLR ')
