#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Tuple

from covapsy_bridge.usb_protocol import LCD_LINE_COUNT, sanitize_lcd_cell

RECOVERY_NONE = 0
RECOVERY_BRAKE = 1
RECOVERY_REVERSE = 2
RECOVERY_REASSESS = 3
RECOVERY_ESCALATE = 4
RECOVERY_FAILSAFE_STOP = 5

ALERT_BRIDGE_STATES = {
    'USB_DISCONNECTED',
    'USB_TIMEOUT',
    'USB_PARSE_ERROR',
}
ALERT_STEERING_STATES = {
    'USB_DISCONNECTED',
    'DXL_NO_RESPONSE',
}
ALERT_RECOVERY_STATES = {
    RECOVERY_BRAKE,
    RECOVERY_REVERSE,
    RECOVERY_REASSESS,
    RECOVERY_ESCALATE,
    RECOVERY_FAILSAFE_STOP,
}


@dataclass
class LcdStatusSnapshot:
    bridge_status: str = 'USB_DISCONNECTED'
    steering_status: str = 'USB_DISCONNECTED'
    car_mode: str = 'IDLE'
    cmd_speed_mps: float = 0.0
    wheel_speed_mps: float = 0.0
    front_clearance_m: float = 0.0
    track_quality_score: float = 0.0
    emergency_brake: bool = False
    safety_veto: bool = False
    recovery_state: int = RECOVERY_NONE
    recovery_attempt: int = 0
    recovery_reason: str = 'none'
    wrong_direction_confidence: float = 0.0
    rear_obstacle: bool = False


def _short_status(status: str) -> str:
    mapping = {
        'RUN': 'RUN',
        'OK': 'OK',
        'WAIT_START': 'WAIT',
        'WATCHDOG_BRAKE': 'WATCHDOG',
        'USB_DISCONNECTED': 'USB_DISC',
        'USB_TIMEOUT': 'USB_TOUT',
        'USB_PARSE_ERROR': 'USB_PARSE',
        'DXL_NO_RESPONSE': 'DXL_NRESP',
    }
    key = str(status or '').strip().upper()
    return mapping.get(key, key[:10] or 'N/A')


def _line(text: str) -> str:
    return sanitize_lcd_cell(text).strip()


def _alert_bridge(snapshot: LcdStatusSnapshot) -> Tuple[str, str, str, str]:
    return (
        _line('ALERT BRIDGE'),
        _line(f'STATE {_short_status(snapshot.bridge_status)}'),
        _line(f'MODE {snapshot.car_mode}'),
        _line(f'VEL {snapshot.wheel_speed_mps:.2f}/{snapshot.cmd_speed_mps:.2f}'),
    )


def _alert_steering(snapshot: LcdStatusSnapshot) -> Tuple[str, str, str, str]:
    return (
        _line('ALERT STEER'),
        _line(f'STATE {_short_status(snapshot.steering_status)}'),
        _line(f'MODE {snapshot.car_mode}'),
        _line(f'REAR {int(snapshot.rear_obstacle)}'),
    )


def _alert_recovery(snapshot: LcdStatusSnapshot) -> Tuple[str, str, str, str]:
    return (
        _line('ALERT RECOVERY'),
        _line(f'STAGE {int(snapshot.recovery_state)} TRY {int(snapshot.recovery_attempt)}'),
        _line(snapshot.recovery_reason),
        _line(f'EB {int(snapshot.emergency_brake)} RO {int(snapshot.rear_obstacle)}'),
    )


def _page_ops_a(snapshot: LcdStatusSnapshot) -> Tuple[str, str, str, str]:
    return (
        _line(f'MODE {snapshot.car_mode}'),
        _line(f'BR {_short_status(snapshot.bridge_status)}'),
        _line(f'ST {_short_status(snapshot.steering_status)}'),
        _line(f'VEL {snapshot.wheel_speed_mps:.2f}/{snapshot.cmd_speed_mps:.2f}'),
    )


def _page_ops_b(snapshot: LcdStatusSnapshot) -> Tuple[str, str, str, str]:
    return (
        _line(f'CLR {snapshot.front_clearance_m:.2f} Q {snapshot.track_quality_score:.2f}'),
        _line(f'EB {int(snapshot.emergency_brake)} VETO {int(snapshot.safety_veto)}'),
        _line(f'WRONG {snapshot.wrong_direction_confidence:.2f}'),
        _line(f'REAR {int(snapshot.rear_obstacle)}'),
    )


def build_lcd_lines(snapshot: LcdStatusSnapshot, page_index: int) -> Tuple[str, str, str, str]:
    bridge_status = str(snapshot.bridge_status or '').strip().upper()
    steering_status = str(snapshot.steering_status or '').strip().upper()
    car_mode = str(snapshot.car_mode or '').strip().upper()
    recovery_state = int(snapshot.recovery_state)

    if bridge_status in ALERT_BRIDGE_STATES:
        lines = _alert_bridge(snapshot)
    elif steering_status in ALERT_STEERING_STATES:
        lines = _alert_steering(snapshot)
    elif car_mode in {'RECOVERY', 'STOPPED'} or recovery_state in ALERT_RECOVERY_STATES:
        lines = _alert_recovery(snapshot)
    elif (int(page_index) % 2) == 0:
        lines = _page_ops_a(snapshot)
    else:
        lines = _page_ops_b(snapshot)

    if len(lines) != LCD_LINE_COUNT:
        raise RuntimeError('LCD formatter returned wrong number of lines')
    return lines
