# CoVAPSy 1/10 autonomous car

Current build of our CoVAPSy 1/10 autonomous car. Bare Python on a Raspberry Pi 5
for perception and control, a small STM32G431 firmware for the actuators and
sensors, and a Webots world that runs the same driving code as the real car. No ROS.

## How it works

```
  RPLIDAR A2M12   USB ----+
  RealSense D435i USB ----+   Raspberry Pi 5            USB-CDC      STM32G431KB (Nucleo, on the Hat)
                          |    race.py (state machine)  /dev/ttyACM0   +- ESC PWM        (PA8)
                          |    gap_follower (driving)   ----------->    +- steering PWM   (PA11)
  trackside laptop  TCP --+    lidar / camera / link                   +- rear IR  ADC   (PA1)
     START / STOP         +------------------------------------------> +- OLED + IMU I2C (PB7/PA15)
                                                                       +- buzzer         (PB6)
```

The work is split between two processors by timing requirement:

- **Raspberry Pi 5** runs the perception and decision loop at 15 Hz: read the
  LiDAR, run follow-the-gap, send one steering and speed command.
- **STM32G431** runs the hard-real-time layer at 50 Hz: generate the servo and
  ESC PWM, read the rear IR, encoder and IMU, drive the OLED, and enforce a
  safety watchdog.

They talk over USB-CDC (`/dev/ttyACM0`) with a short ASCII line protocol (see
[`stm32/README.md`](stm32/README.md)). The watchdog is independent: if the Pi
stops sending for 250 ms, the STM32 neutralises the outputs on its own.

The driving logic is one pure-Python function, `common/gap_follower.py`. The Pi
runtime (`pi/race.py`) and the Webots controller both import it unchanged, so the
simulator and the car run identical code. Every tunable constant is in
`common/config.py`.

Follow-the-gap pipeline: sanitise the scan, inflate obstacles by half the car
width (disparity extension), carve a safety bubble around the nearest point, pick
the widest gap, aim between its centre and its deepest point, set speed from
steering angle and time-to-collision, and reverse out of dead ends when the rear
is clear.

## Layout

| Dir | Contents |
|---|---|
| `common/` | Shared driving code: `gap_follower.py` (follow-the-gap) and `config.py` (all tunables). Imported by both the car and the simulator. |
| `pi/` | On-car runtime. See [`pi/README.md`](pi/README.md): `race.py`, `lidar.py`, `camera.py`, `stm32_link.py`, `covapsy_ctl.py`. |
| `tests/` | `test_gap_follower.py` (pure unit test, runs anywhere) and hardware smoke tests (`test_lidar/motor/steering/camera.py`, `steering_calibration.py`, run on the Pi). |
| `simulation/` | Webots R2025a world plus the `covapsy_sim` controller. See [`simulation/README.md`](simulation/README.md). |
| `stm32/` | Firmware. See [`stm32/README.md`](stm32/README.md): motor and steering PWM, USB-CDC command and telemetry, rear-IR ADC, BNO055 IMU, OLED, buzzer, watchdog. |

## Run

**Firmware** (build and flash from the Pi, over the same USB cable):
```bash
cd stm32 && make && make flash
```

**Drive a race** (wheels off the ground for first runs):
```bash
cd pi
python3 race.py                 # boots into IDLE, waits for a START signal
python3 covapsy_ctl.py start    # go. also: stop | status
```

**Tests**:
```bash
python3 -m pytest tests/test_gap_follower.py   # pure logic, any machine
python3 tests/test_lidar.py                    # on the Pi (live LiDAR)
python3 tests/test_steering.py                 # on the Pi, wheels off ground
python3 tests/test_motor.py                    # on the Pi, wheels off ground
```

**Simulation**: open `simulation/worlds/standalone.wbt` in Webots R2025a, press
Play, click the 3D view, then `A` to drive and `N` to stop.

## Sensor status

| Sensor | Use | Status |
|---|---|---|
| RPLIDAR A2M12 (USB) | follow-the-gap steering and obstacle avoidance | working |
| Rear IR (Sharp, STM32 ADC PA1) | reverse only when the rear is clear | working |
| BNO055 IMU (STM32 I2C 0x28) | heading telemetry on the OLED | working |
| RealSense D435i RGB (UVC `/dev/video4`) | wrong-way detection (advisory, not in the control loop) | working |

The LiDAR field of view is limited to the front (Pi and battery occlude the rear).
