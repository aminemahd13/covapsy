# Raspberry Pi 5 Setup

## Target Platform
- Board: Raspberry Pi 5 (2GB).
- OS: Ubuntu Server 24.04.
- ROS: Jazzy.
- Power: stable 5V/5A USB-C input.

## 1. Base System Provisioning
On Pi:
```bash
cd ~/mycar
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

`setup_pi.sh` covers:
- system update
- swap/zram
- base tools
- udev rules
- boot config append (`uart0`, `spi`, USB current)
- Python deps (`pyserial`, `spidev`, `rpi-hardware-pwm`)

`install_ros2.sh` covers:
- ROS2 apt repo setup
- Jazzy base + required ROS packages
- rosdep init/update
- shell environment exports

## 2. Deploy Workspace to Pi
From development machine:
```bash
bash scripts/deploy_to_pi.sh <pi_host_or_ip>
```

On Pi:
```bash
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

## 3. Backend Selection and Boot Requirements
- `backend:=spi`:
  - `dtparam=spi=on`
  - HAT jumpers route PWM source to STM32
- `backend:=uart`:
  - `dtparam=uart0=on`
  - HAT jumpers route PWM source to STM32
- `backend:=pi_pwm`:
  - HAT jumpers route PWM source to Raspberry Pi
  - `rpi-hardware-pwm` installed

Parameters live in:
- `ros2_ws/src/covapsy_bringup/config/backend_params.yaml`

## 4. Bringup Sequence
First safe run (wheels off ground):
```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=spi
```

Then full stack:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE
```

Competition control signals:
```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```
In competition defaults, stop is latched; relaunch for a fresh run after `/race_stop`.

Default learning -> racing behavior in `car_full.launch.py`:
- starts in `IDLE`
- `/race_start` switches to `LEARNING`
- tries direction-aware saved-path reuse from `~/.ros/covapsy`
- if missing, track learning runs for required setup laps (`track_learning_required_laps:=2`)
- auto-switches to `RACING` after `/track_learned=true` and handoff confirm delay

Monitor:
```bash
ros2 topic echo /car_mode
ros2 topic echo /track_learned
ros2 topic echo /saved_track_loaded
ros2 topic echo /track_direction
ros2 topic echo /mcu_status --once
```

Backend switch examples:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=uart
ros2 launch covapsy_bringup car_full.launch.py backend:=pi_pwm
```

## 5. Runtime Sanity Checks
```bash
ros2 topic hz /scan
ros2 topic echo /wheel_speed --once
ros2 topic echo /rear_obstacle --once
ros2 topic echo /mcu_status --once
ros2 topic echo /car_mode --once
```

Expected:
- `/scan` is stable at LiDAR rate.
- `/mcu_status` includes selected backend and no init error.
- `/car_mode` matches requested initial mode and transitions on start/stop topics.

Complete operator procedure:
- [Operations Runbook](OPERATIONS.md)
