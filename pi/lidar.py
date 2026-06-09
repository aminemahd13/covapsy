"""
RPLIDAR A2M12 driver producing the canonical COVAPSY scan.

Scan: list of 360 floats, distances in metres, index = angle in degrees.
Index 0 = car front, increasing index = counter-clockwise (left), so index 90
= 90 deg left, index 270 = 90 deg right. 0.0 means no return (treated as max
range downstream).

A daemon thread reads scans into a shared 360-float array under a lock;
get_scan() returns a thread-safe copy. On error the thread resets the serial
buffers, reconnects, and resumes.
"""

import threading
import time

from rplidar import RPLidar, RPLidarException

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 256000

_HEALTHY_AGE_S = 0.5  # a scan older than this is not healthy


class Lidar:
    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 front_offset_deg: int = 0):
        self.port = port
        self.baud = baud
        self.front_offset_deg = int(front_offset_deg)
        self._lidar = None
        self._scan_m = [0.0] * 360          # metres array
        self._lock = threading.Lock()
        self._last_update = 0.0             # time.monotonic() of last good scan
        self._running = False
        self._thread = None

    # lifecycle

    def start(self) -> "Lidar":
        """Connect, start the motor, and launch the background reader."""
        self._lidar = RPLidar(self.port, baudrate=self.baud)
        self._lidar.connect()
        self._lidar.start_motor()
        time.sleep(1.0)  # let the motor reach speed before reading
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """Stop the reader thread and motor; connection stays open."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
            except RPLidarException:
                pass

    def close(self) -> None:
        """Stop, then disconnect the serial port."""
        self.stop()
        if self._lidar is not None:
            try:
                self._lidar.disconnect()
            finally:
                self._lidar = None

    def __enter__(self) -> "Lidar":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # public read API

    def get_scan(self) -> list:
        """Thread-safe copy of the latest 360-element metres array."""
        with self._lock:
            return list(self._scan_m)

    @property
    def healthy(self) -> bool:
        """True if a scan arrived within the last _HEALTHY_AGE_S seconds."""
        with self._lock:
            last = self._last_update
        return last > 0.0 and (time.monotonic() - last) < _HEALTHY_AGE_S

    # background reader

    def _store_scan(self, scan) -> None:
        """Map one raw express scan into the metres array.

        Each point is (quality, angle_deg, dist_mm). Flip (359 - angle) so
        increasing index is CCW/left, then add front_offset_deg to set which
        raw angle is the car front. Beams not refreshed this rotation keep
        their previous value.
        """
        with self._lock:
            for _quality, angle_deg, dist_mm in scan:
                idx = (359 - int(angle_deg) + self.front_offset_deg) % 360
                self._scan_m[idx] = dist_mm / 1000.0
            self._last_update = time.monotonic()

    def _read_loop(self) -> None:
        while self._running:
            try:
                for scan in self._lidar.iter_scans(scan_type='express'):
                    if not self._running:
                        break
                    self._store_scan(scan)
            except RPLidarException as exc:
                # Serial hiccup or descriptor mismatch: reset and resume.
                print(f"[lidar] {exc!r} -- resetting and reconnecting")
                self._recover()
            except Exception as exc:  # never let the thread die silently
                print(f"[lidar] unexpected {exc!r} -- resetting and reconnecting")
                self._recover()

    def _recover(self) -> None:
        """Clear device and serial buffers, then reconnect with backoff."""
        if not self._running or self._lidar is None:
            return
        try:
            self._lidar.stop()
            self._lidar.clean_input()
        except Exception:
            pass
        try:
            self._lidar.disconnect()
        except Exception:
            pass
        time.sleep(0.5)
        try:
            self._lidar.connect()
            self._lidar.start_motor()
            time.sleep(1.0)
        except Exception as exc:
            print(f"[lidar] reconnect failed: {exc!r} -- retrying")
            time.sleep(1.0)
