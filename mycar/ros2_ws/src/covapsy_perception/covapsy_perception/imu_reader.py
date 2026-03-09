"""BNO055 IMU reader for the CoVAPSy Mezzanine board.

Reads yaw rate (gyroscope), linear acceleration, and absolute
orientation from the BNO055 over I2C.  Pure Python, no ROS2 dependency.

Hardware:  BNO055 on Mezzanine_CoVASPSy_v1re2 (U4), I2C address 0x28.
Reference: Covapsy-prof/Software/programmes_capteurs_afficheur_AX12/test_bno055.py

Usage on Raspberry Pi:
    reader = BNO055Reader()
    reader.initialise()
    while True:
        data = reader.read()
        print(data.yaw_rate, data.lat_accel, data.lon_accel)
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional

# ── BNO055 Register Map ──
_ADDRESS = 0x28          # 7-bit I2C address

_REG_PAGE_SWAP = 0x07
_REG_ACC_CONF = 0x08
_REG_MAG_CONF = 0x09
_REG_GYR_CONF_0 = 0x0A
_REG_GYR_CONF_1 = 0x0B
_REG_HEADING = 0x1A
_REG_EULER_H = 0x1A      # Euler heading LSB (2 bytes)
_REG_EULER_R = 0x1C      # Euler roll LSB (2 bytes)
_REG_EULER_P = 0x1E      # Euler pitch LSB (2 bytes)
_REG_GYRO_X = 0x14       # Gyro X LSB (2 bytes, rad/s × 900)
_REG_GYRO_Y = 0x16
_REG_GYRO_Z = 0x18
_REG_LIN_ACCEL_X = 0x28  # Linear accel X LSB (2 bytes, m/s² × 100)
_REG_LIN_ACCEL_Y = 0x2A
_REG_LIN_ACCEL_Z = 0x2C
_REG_CALIB_STAT = 0x35
_REG_TEMP = 0x34
_REG_UNIT_SEL = 0x3B
_REG_PWR_MODE = 0x3E
_REG_MODE = 0x3D
_REG_SYS_TRIGGER = 0x3F
_REG_TEMP_SOURCE = 0x40

# Operating modes
_MODE_CONFIG = 0x00
_MODE_NDOF = 0x0C        # Full NDOF fusion (9-DOF)

# Unit selections — rad/s for gyro, m/s² for accel, radians for euler
_UNIT_SEL_RADIAN_MPS2 = 0x06


@dataclass
class IMUReading:
    """Timestamped IMU measurement."""
    yaw_rate: float = 0.0        # rad/s — positive = turning left (Z-axis gyro)
    lat_accel: float = 0.0       # m/s²  — positive = leftward (Y-axis linear accel)
    lon_accel: float = 0.0       # m/s²  — positive = forward  (X-axis linear accel)
    heading_rad: float = 0.0     # absolute heading (rad)
    roll_rad: float = 0.0        # roll angle (rad)
    pitch_rad: float = 0.0       # pitch angle (rad)
    temperature_c: float = 0.0
    calibration: int = 0         # BNO055 calibration status byte
    timestamp: float = 0.0
    valid: bool = False


class BNO055Reader:
    """Read BNO055 IMU data over I2C (smbus2).

    Designed for the CoVAPSy Mezzanine board.
    Falls back gracefully if I2C is unavailable (e.g. on development PC).
    """

    def __init__(self, bus: int = 1, address: int = _ADDRESS) -> None:
        self._bus_num = bus
        self._address = address
        self._bus = None          # type: ignore[assignment]
        self._available = False
        self._last_heading = 0.0

    @property
    def available(self) -> bool:
        return self._available

    def initialise(self) -> bool:
        """Open I2C bus and configure BNO055 in NDOF fusion mode.

        Returns True on success, False if hardware is unavailable.
        """
        try:
            import smbus2
            self._bus = smbus2.SMBus(self._bus_num)
        except (ImportError, OSError):
            self._available = False
            return False

        try:
            # Reset
            self._bus.write_byte_data(self._address, _REG_SYS_TRIGGER, 0x20)
            time.sleep(0.8)

            # Page 1: sensor config
            self._bus.write_byte_data(self._address, _REG_PAGE_SWAP, 1)
            self._bus.write_byte_data(self._address, _REG_ACC_CONF, 0x08)
            self._bus.write_byte_data(self._address, _REG_GYR_CONF_0, 0x23)
            self._bus.write_byte_data(self._address, _REG_GYR_CONF_1, 0x00)
            self._bus.write_byte_data(self._address, _REG_MAG_CONF, 0x1B)

            # Page 0: operating config
            self._bus.write_byte_data(self._address, _REG_PAGE_SWAP, 0)
            self._bus.write_byte_data(self._address, _REG_TEMP_SOURCE, 0x01)
            # Units: radians for gyro+euler, m/s² for accel
            self._bus.write_byte_data(self._address, _REG_UNIT_SEL, _UNIT_SEL_RADIAN_MPS2)
            self._bus.write_byte_data(self._address, _REG_PWR_MODE, 0x00)
            self._bus.write_byte_data(self._address, _REG_MODE, _MODE_NDOF)
            time.sleep(0.05)

            self._available = True
            return True
        except OSError:
            self._available = False
            return False

    def read(self) -> IMUReading:
        """Read all IMU data in one shot.  ~0.5 ms on I2C-1."""
        if not self._available or self._bus is None:
            return IMUReading(timestamp=time.monotonic())

        try:
            now = time.monotonic()

            # Read 6 bytes of gyro (X, Y, Z) — rad/s with 900 LSB/rad/s
            gyro_raw = self._bus.read_i2c_block_data(self._address, _REG_GYRO_X, 6)
            gx = struct.unpack_from('<h', bytes(gyro_raw), 0)[0] / 900.0
            gy = struct.unpack_from('<h', bytes(gyro_raw), 2)[0] / 900.0
            gz = struct.unpack_from('<h', bytes(gyro_raw), 4)[0] / 900.0

            # Read 6 bytes of linear acceleration (X, Y, Z) — m/s² with 100 LSB/(m/s²)
            acc_raw = self._bus.read_i2c_block_data(self._address, _REG_LIN_ACCEL_X, 6)
            ax = struct.unpack_from('<h', bytes(acc_raw), 0)[0] / 100.0
            ay = struct.unpack_from('<h', bytes(acc_raw), 2)[0] / 100.0

            # Euler angles (heading, roll, pitch) — rad with 900 LSB/rad
            euler_raw = self._bus.read_i2c_block_data(self._address, _REG_EULER_H, 6)
            heading = struct.unpack_from('<h', bytes(euler_raw), 0)[0] / 900.0
            roll = struct.unpack_from('<h', bytes(euler_raw), 2)[0] / 900.0
            pitch = struct.unpack_from('<h', bytes(euler_raw), 4)[0] / 900.0

            # Calibration + temperature
            calib = self._bus.read_byte_data(self._address, _REG_CALIB_STAT)
            temp = self._bus.read_byte_data(self._address, _REG_TEMP)

            return IMUReading(
                yaw_rate=gz,            # Z-axis gyro = yaw rate
                lat_accel=ay,           # Y-axis = lateral
                lon_accel=ax,           # X-axis = longitudinal
                heading_rad=heading,
                roll_rad=roll,
                pitch_rad=pitch,
                temperature_c=float(temp),
                calibration=calib,
                timestamp=now,
                valid=True,
            )
        except OSError:
            return IMUReading(timestamp=time.monotonic())

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
            self._available = False
