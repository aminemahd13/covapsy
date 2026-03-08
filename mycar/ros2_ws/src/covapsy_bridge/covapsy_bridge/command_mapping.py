"""Backend-agnostic drive mapping helpers for the COVAPSY bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveLimits:
    max_steering_deg: float = 18.0
    max_speed_fwd: float = 2.0
    max_speed_rev: float = -4.0


@dataclass(frozen=True)
class PwmCalibration:
    prop_stop: float = 7.5
    prop_deadband: float = 0.4
    prop_delta_max: float = 1.5
    speed_hard: float = 8.0
    direction_sign: float = -1.0
    angle_deg_max: float = 18.0
    angle_min: float = 5.5
    angle_max: float = 9.3
    angle_center: float = 7.4


def clamp_drive_command(
    steering_rad: float, speed_m_s: float, limits: DriveLimits
) -> tuple[float, float]:
    """Clamp and convert incoming command to steering degrees + speed m/s."""
    steering_deg = math.degrees(float(steering_rad))
    steering_deg = max(-limits.max_steering_deg, min(limits.max_steering_deg, steering_deg))
    speed = max(limits.max_speed_rev, min(limits.max_speed_fwd, float(speed_m_s)))
    return steering_deg, speed


def encode_signed_to_u8(value: float, min_v: float, max_v: float) -> int:
    """Map a value in [min_v, max_v] to uint8 [0, 255]."""
    if max_v <= min_v:
        return 127
    clipped = max(min_v, min(max_v, float(value)))
    norm = (clipped - min_v) / (max_v - min_v)
    return int(round(norm * 255.0))


def decode_u8_to_signed(byte_val: int, min_v: float, max_v: float) -> float:
    """Map uint8 [0, 255] to value in [min_v, max_v]."""
    b = max(0, min(255, int(byte_val)))
    norm = b / 255.0
    return min_v + norm * (max_v - min_v)


def build_spi_frame_6b(
    steering_deg: float,
    speed_m_s: float,
    limits: DriveLimits,
    header0: int = 0x55,
    header1: int = 0x55,
    steering_index: int = 2,
    speed_index: int = 3,
    flags_index: int = 4,
) -> bytes:
    """Build default 6-byte SPI frame [H0,H1,steer,speed,flags,checksum]."""
    frame = [0] * 6
    frame[0] = header0 & 0xFF
    frame[1] = header1 & 0xFF
    frame[steering_index] = encode_signed_to_u8(
        steering_deg, -limits.max_steering_deg, limits.max_steering_deg
    )
    frame[speed_index] = encode_signed_to_u8(speed_m_s, limits.max_speed_rev, limits.max_speed_fwd)
    frame[flags_index] = 0
    frame[5] = sum(frame[:5]) & 0xFF
    return bytes(frame)


def parse_spi_frame_6b(
    frame: bytes,
    limits: DriveLimits,
    header0: int = 0x55,
    header1: int = 0x55,
    speed_index: int = 3,
    flags_index: int = 4,
) -> tuple[float, bool]:
    """Parse default 6-byte SPI response as (wheel_speed_m_s, rear_obstacle)."""
    if len(frame) != 6:
        return 0.0, False
    if frame[0] != (header0 & 0xFF) or frame[1] != (header1 & 0xFF):
        return 0.0, False
    checksum = sum(frame[:5]) & 0xFF
    if frame[5] != checksum:
        return 0.0, False
    speed = decode_u8_to_signed(frame[speed_index], limits.max_speed_rev, limits.max_speed_fwd)
    rear = bool(frame[flags_index] & 0x01)
    return speed, rear


def crc8(data: bytes) -> int:
    """CRC8 poly 0x07."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def build_uart_drive_frame(steering_deg: float, speed_m_s: float) -> bytes:
    """Build UART drive frame [0xAA,0x55,0x01,<ff>,CRC8]."""
    import struct

    payload = struct.pack("<ff", float(steering_deg), float(speed_m_s))
    frame = bytes([0xAA, 0x55, 0x01]) + payload
    return frame + bytes([crc8(frame)])


def pwm_duties_from_command(speed_m_s: float, steering_deg: float, cal: PwmCalibration) -> tuple[float, float]:
    """Map speed + steering to propulsion and steering duty cycles."""
    speed = max(-cal.speed_hard, min(cal.speed_hard, float(speed_m_s)))
    steer = max(-cal.angle_deg_max, min(cal.angle_deg_max, float(steering_deg)))

    if speed == 0.0:
        prop = cal.prop_stop
    elif speed > 0.0:
        delta = (speed * cal.prop_delta_max) / cal.speed_hard
        prop = cal.prop_stop + cal.direction_sign * (cal.prop_deadband + delta)
    else:
        delta = (speed * cal.prop_delta_max) / cal.speed_hard
        prop = cal.prop_stop - cal.direction_sign * (cal.prop_deadband - delta)

    steer_pwm = cal.angle_center + cal.direction_sign * (
        (cal.angle_max - cal.angle_min) * steer / (2.0 * cal.angle_deg_max)
    )
    steer_pwm = max(cal.angle_min, min(cal.angle_max, steer_pwm))
    return prop, steer_pwm

