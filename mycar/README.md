# COVAPSY Final Project

Self-contained COVAPSY stack for:
1. ROS2-in-the-loop Webots simulation (primary development path).
2. Raspberry Pi 5 deployment with pluggable actuator backend (`spi`, `uart`, `pi_pwm`).

This project is organized so the same autonomy nodes run in simulation and on the real car with the same ROS topics.

## What Is Included
- `ros2_ws/src/covapsy_perception`: LiDAR and camera preprocessing.
- `ros2_ws/src/covapsy_nav`: reactive gap follower, pure pursuit, mode controller.
- `ros2_ws/src/covapsy_bridge`: backend-selectable bridge for actuation/telemetry.
- `ros2_ws/src/covapsy_bringup`: launch files, parameters, SLAM config.
- `simulation/webots`: self-contained worlds, protos, controllers.
- `docs`: operator documentation.

## Quick Start (Standalone Webots on Windows, No ROS)
Prerequisites:
- Windows 10/11.
- Webots R2025a installed.

Run from PowerShell:
```powershell
cd mycar
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1
```

World/controller used:
- `simulation/webots/worlds/Piste_CoVAPSy_2025a_standalone.wbt`
- `simulation/webots/controllers/covapsy_standalone/covapsy_standalone.py`

Keyboard controls:
- `A`: start autonomous mode
- `N`: stop autonomous mode
- `S`: toggle advanced/simple algorithm
- `R`: reverse maneuver
- `+` / `-`: adjust speed cap

## Quick Start (ROS2 Simulation on Ubuntu)
Prerequisites:
- Ubuntu 24.04 + ROS2 Jazzy.
- Webots R2025a.

Build:
```bash
cd mycar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run ROS stack:
```bash
ros2 launch covapsy_bringup sim_webots.launch.py
```

Start Webots from the same sourced terminal:
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

Expected checks:
```bash
ros2 topic hz /scan            # ~10 Hz or Webots lidar rate
ros2 topic hz /scan_filtered   # follows /scan
ros2 topic echo /car_mode --once
ros2 topic echo /mcu_status --once
```

## Quick Start (Raspberry Pi 5)
On Pi:
```bash
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

From development machine:
```bash
bash scripts/deploy_to_pi.sh <pi_ip_or_hostname>
```

Back on Pi:
```bash
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

Safe first run:
```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=spi
```

Full run:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE
```

## Backends
- `spi` (default): Pi master SPI <-> STM32 slave, STM32 generates PWM.
- `uart`: Pi UART <-> STM32 UART, STM32 generates PWM.
- `pi_pwm`: Pi directly generates ESC/servo PWM (HAT jumpers must route PWM from Pi).

## Main Launch Files
- `sim_webots.launch.py`: simulation stack, no hardware bridge.
- `car_safe.launch.py`: minimal real-car bringup for first on-track tests.
- `car_full.launch.py`: complete real-car stack.
- `slam.launch.py`: SLAM toolbox mapping/localization.

## Documentation
- [Architecture](docs/ARCHITECTURE.md)
- [Simulation](docs/SIMULATION.md)
- [Pi 5 Setup](docs/PI5_SETUP.md)
- [HAT Jumpers](docs/HAT_JUMPERS.md)
- [Calibration](docs/CALIBRATION.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Validation](docs/VALIDATION.md)
