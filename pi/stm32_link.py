"""
pyserial wrapper around the STM32 firmware.

Write protocol (line-based ASCII, newline-terminated):
    S<deg>\n   steering, degrees, clamped [-18, +18]
    V<mps>\n   speed, m/s, clamped [-V_MAX_HARD, +V_MAX_SOFT]
    X\n        full stop (speed 0, steering 0)
Read protocol (telemetry pushed ~2 Hz):
    HB <ms> prop=<us> dir=<us> oled=<yes|no> rear=<raw> rblk=<0|1> head=<deg>
       enc=<pulses/s> spd=<mm/s> tot=<pulse count>
STM32 watchdog: 250 ms of silence gives neutral output, so call at >= 4 Hz.
"""

import collections
import threading
import time

import serial

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

_TELEM_STALE_S = 1.0   # telemetry older than this is treated as unknown (s)


class STM32Link:
    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._lock = threading.Lock()

        # Latest telemetry, updated by the reader thread.
        self._t_lock = threading.Lock()
        self._rear_raw = 0
        self._rear_blocked = None          # None = never heard from firmware
        self._heading_deg = None
        self._wheel_pps = None             # encoder pulses/sec
        self._speed_mps = 0.0              # measured wheel speed (m/s)
        self._telem_ts = 0.0
        # Raw firmware text lines for display by tests/tools. Bounded so it
        # does not grow unbounded when nobody pops it.
        self._line_buf = collections.deque(maxlen=300)

        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # write API

    def set_steering(self, deg: float) -> None:
        self._send(f"S{deg:+.2f}\n")

    def set_speed(self, mps: float) -> None:
        self._send(f"V{mps:+.2f}\n")

    def stop(self) -> None:
        self._send("X\n")

    def send_raw(self, line: str) -> None:
        """Send a raw firmware command line (newline added), e.g. 'P1800', 'WD0', 'REV'."""
        self._send(line + "\n")

    def _send(self, s: str) -> None:
        with self._lock:
            self.ser.write(s.encode("ascii"))

    # telemetry API

    def rear_clear(self) -> bool:
        """True only if fresh telemetry says the rear is clear. Stale or never-heard returns False."""
        with self._t_lock:
            if self._rear_blocked is None:
                return False
            if (time.monotonic() - self._telem_ts) > _TELEM_STALE_S:
                return False
            return self._rear_blocked == 0

    @property
    def rear_raw(self) -> int:
        with self._t_lock:
            return self._rear_raw

    @property
    def heading_deg(self):
        """Fused IMU heading in degrees, or None if absent or stale."""
        with self._t_lock:
            if self._heading_deg is None or self._heading_deg < 0:
                return None
            if (time.monotonic() - self._telem_ts) > _TELEM_STALE_S:
                return None
            return self._heading_deg

    @property
    def speed_mps(self):
        """Measured wheel speed (m/s) from the encoder, or None if stale."""
        with self._t_lock:
            if (time.monotonic() - self._telem_ts) > _TELEM_STALE_S:
                return None
            return self._speed_mps

    @property
    def wheel_pps(self) -> int:
        """Latest encoder pulses/sec (0 if never heard)."""
        with self._t_lock:
            return self._wheel_pps or 0

    def wheels_stalled(self, min_pps: int = 5) -> bool:
        """True only if fresh encoder telemetry shows the wheels below min_pps. Stale or never-heard returns False."""
        with self._t_lock:
            if self._wheel_pps is None:
                return False
            if (time.monotonic() - self._telem_ts) > _TELEM_STALE_S:
                return False
            return self._wheel_pps < min_pps

    @property
    def telemetry_age(self) -> float:
        with self._t_lock:
            if self._telem_ts == 0.0:
                return float("inf")
            return time.monotonic() - self._telem_ts

    def pop_lines(self):
        """Return and clear raw STM32 text lines received since the last call (the reader thread owns the port)."""
        with self._t_lock:
            lines = list(self._line_buf)
            self._line_buf.clear()
            return lines

    # reader thread

    def _read_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                chunk = self.ser.read(128)
            except Exception:
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                with self._t_lock:
                    self._line_buf.append(line)
                self._parse_line(line)

    def _parse_line(self, line: str) -> None:
        if not line.startswith("HB "):
            return
        rear_raw = rblk = head = enc = spd = None
        for tok in line.split():
            if tok.startswith("rear="):
                rear_raw = _safe_int(tok[5:])
            elif tok.startswith("rblk="):
                rblk = _safe_int(tok[5:])
            elif tok.startswith("head="):
                head = _safe_int(tok[5:])
            elif tok.startswith("enc="):
                enc = _safe_int(tok[4:])
            elif tok.startswith("spd="):
                spd = _safe_int(tok[4:])
        with self._t_lock:
            if rear_raw is not None:
                self._rear_raw = rear_raw
            if rblk is not None:
                self._rear_blocked = rblk
            if head is not None:
                self._heading_deg = head
            if enc is not None:
                self._wheel_pps = enc
            if spd is not None:
                self._speed_mps = spd / 1000.0
            self._telem_ts = time.monotonic()

    # lifecycle

    def close(self) -> None:
        self._running = False
        try:
            self.stop()
        finally:
            self.ser.close()

    def __enter__(self) -> "STM32Link":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def _safe_int(s: str):
    try:
        return int(s)
    except ValueError:
        return None
