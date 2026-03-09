from covapsy_bridge.command_mapping import DriveLimits
from covapsy_bridge.command_mapping import PwmCalibration
from covapsy_bridge.command_mapping import build_spi_frame_6b
from covapsy_bridge.command_mapping import clamp_drive_command
from covapsy_bridge.command_mapping import parse_spi_frame_6b
from covapsy_bridge.command_mapping import pwm_duties_from_command


def test_clamp_drive_command_limits_values():
    limits = DriveLimits(max_steering_deg=18.0, max_speed_fwd=2.0, max_speed_rev=-4.0)
    steering_deg, speed = clamp_drive_command(steering_rad=10.0, speed_m_s=10.0, limits=limits)
    assert steering_deg == 18.0
    assert speed == 2.0


def test_spi_frame_roundtrip_parse():
    limits = DriveLimits(max_steering_deg=18.0, max_speed_fwd=2.0, max_speed_rev=-4.0)
    tx = build_spi_frame_6b(steering_deg=0.0, speed_m_s=1.0, limits=limits, command_flags=0x00)
    speed, rear = parse_spi_frame_6b(tx, limits=limits)
    assert abs(speed - 1.0) < 0.2
    assert rear is False


def test_spi_frame_includes_command_flags_byte():
    limits = DriveLimits()
    tx = build_spi_frame_6b(steering_deg=0.0, speed_m_s=0.0, limits=limits, command_flags=0xA5)
    assert tx[4] == 0xA5


def test_pwm_mapping_stays_in_bounds():
    cal = PwmCalibration()
    prop, steer = pwm_duties_from_command(speed_m_s=2.0, steering_deg=18.0, cal=cal)
    assert 4.0 <= prop <= 11.0
    assert cal.angle_min <= steer <= cal.angle_max
