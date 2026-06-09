# pi

Bare-Python runtime for the car. Talks to the STM32 over USB-CDC
(`/dev/ttyACM0`), the RPLIDAR over USB (`/dev/ttyUSB0`), and optionally the
RealSense RGB over a UVC node (`/dev/video4`).

## Programs

| File | Purpose |
|---|---|
| `race.py` | Race control. State machine IDLE, RUNNING, STOPPED, with remote start/stop, rear-IR-gated reverse, stuck recovery, and optional camera wrong-way. Boots into IDLE. `--dry-run` streams steering with throttle forced to 0. |
| `covapsy_ctl.py` | Client that sends START, STOP, STATUS to `race.py`. |
| `lidar.py` | RPLIDAR A2M12 driver. Produces the canonical 360-sample scan in metres. |
| `stm32_link.py` | STM32 link: write S/V/X commands, read HB telemetry (rear IR, heading). |
| `camera.py` | RealSense D435i RGB via OpenCV. Border-colour wrong-way (advisory). |

The driving code (`gap_follower.py`) and tunables (`config.py`) live in
`../common/` and are shared with the Webots simulator. Tests are in `../tests/`.

## Running a race (wheels off the ground for first runs)

```bash
python3 race.py                 # boots IDLE, waits for START
python3 race.py --dry-run       # steering tracks LiDAR, throttle forced 0
python3 race.py --camera        # also read RGB wrong-way (advisory)
```

Start and stop it (the only allowed team-to-car signals):

```bash
python3 covapsy_ctl.py start    # IDLE/STOPPED to RUNNING
python3 covapsy_ctl.py stop     # to STOPPED (motors neutral)
python3 covapsy_ctl.py status
# from the trackside laptop:  python3 covapsy_ctl.py start --host <pi-ip>
# SSH fallback on the Pi:     touch /tmp/covapsy/start  (or stop / quit)
```

Safety: the STM32 has a 250 ms comms watchdog. If `race.py` exits or USB drops,
the car neutralises on its own. `race.py` always boots into IDLE and never drives
until a START.

## Dependencies

`pyserial` (link), `rplidar-roboticia` (LiDAR), `python3-opencv` + `v4l-utils`
(camera, via apt). See `requirements.txt`.
