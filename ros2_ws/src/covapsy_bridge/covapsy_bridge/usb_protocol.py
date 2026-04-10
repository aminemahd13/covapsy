#!/usr/bin/env python3
import math
from typing import Tuple


def bool_to_wire(value: bool) -> str:
    return '1' if value else '0'


def parse_wire_bool(token: str) -> bool:
    value = token.strip().lower()
    if value in ('1', 'true', 't', 'yes', 'y', 'on'):
        return True
    if value in ('0', 'false', 'f', 'no', 'n', 'off'):
        return False
    raise ValueError(f'invalid bool token: {token!r}')


def encode_command_line(
    *,
    seq: int,
    steer_deg: float,
    speed_mps: float,
    run_enable: bool,
    emergency_brake: bool,
) -> str:
    return (
        f'CMD,{int(seq)},{float(steer_deg):.3f},{float(speed_mps):.3f},'
        f'{bool_to_wire(run_enable)},{bool_to_wire(emergency_brake)}\n'
    )


def parse_telemetry_line(line: str) -> Tuple[int, float, bool, int]:
    parts = [p.strip() for p in line.strip().split(',')]
    if len(parts) != 5:
        raise ValueError(f'wrong token count: {len(parts)}')
    if parts[0] != 'TEL':
        raise ValueError(f'wrong prefix: {parts[0]!r}')

    seq = int(parts[1])
    wheel_speed = float(parts[2])
    rear_obstacle = parse_wire_bool(parts[3])
    status_code = int(parts[4])

    if not math.isfinite(wheel_speed):
        raise ValueError('wheel speed is not finite')

    return seq, wheel_speed, rear_obstacle, status_code
