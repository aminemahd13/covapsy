# CoVAPSy 2026 Workspace

End-to-end repository for a CoVAPSy-style autonomous 1/10 car stack:

- ROS2 Jazzy autonomy pipeline (reactive, learning, race, recovery)
- Webots simulation assets and controllers
- STM32 HAT firmware workspace (protocol + control mapping)
- Deployment and setup scripts for Raspberry Pi 5

The repository is organized so you can develop in simulation, run the ROS graph on Linux/Pi, and integrate with STM32 firmware for real-car operation.

## Table of Contents

- [What This Repo Contains](#what-this-repo-contains)
- [System Architecture](#system-architecture)
- [Repository Layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Quick Start (ROS2 Workspace)](#quick-start-ros2-workspace)
- [How To Run](#how-to-run)
- [How It Works (Node-by-Node)](#how-it-works-node-by-node)
- [Configuration Profiles](#configuration-profiles)
- [Simulation](#simulation)
- [STM32 Firmware](#stm32-firmware)
- [Deployment to Raspberry Pi](#deployment-to-raspberry-pi)
- [Debugging and Validation](#debugging-and-validation)
- [Known Integration Notes](#known-integration-notes)

## What This Repo Contains

This codebase implements a layered control model:

- LEARN mode: reactive LiDAR driving is primary.
- RACE mode: pure pursuit is nominal.
- Reactive safety overlay (RACE): can cap speed, veto unsafe steering, and trigger braking.
- Recovery FSM: bounded escape sequence for stuck/wrong-way situations.
- Bridge layer: final low-level command limiting, start/stop gating, and watchdog behavior.

Direction/color detection is used as semantic state (direction confidence and wrong-way detection), not as direct steering authority.

## System Architecture

High-level flow:

1. Sensors publish `/scan`, `/odom`, `/image_raw`, `/imu/data`.
2. LiDAR scan is filtered.
3. Reactive and pursuit controllers each produce candidate drive commands.
4. `mode_controller_node` selects nominal authority by mode and applies safety/recovery logic.
5. `stm32_bridge_node` enforces start/stop and watchdog, then forwards limited command downstream.

Core command chain:

- Reactive command: `/cmd_drive_reactive`
- Pursuit command: `/cmd_drive_pursuit`
- Final autonomy command: `/cmd_drive`
- Low-level bridge output: `/stm32/cmd_drive`

## Repository Layout

- `ros2_ws/`: ROS2 workspace (source + build/install/log)
- `ros2_ws/src/covapsy_bringup/`: launch files and mode configs
- `ros2_ws/src/covapsy_interfaces/`: custom ROS messages
- `ros2_ws/src/covapsy_lidar/`: scan filter + reactive driver
- `ros2_ws/src/covapsy_direction/`: direction/wrong-way detector
- `ros2_ws/src/covapsy_learning/`: track learner and quality scoring
- `ros2_ws/src/covapsy_control/`: pure pursuit + mode controller + recovery FSM
- `ros2_ws/src/covapsy_bridge/`: STM32 bridge node
- `ros2_ws/src/covapsy_tools/`: telemetry logger and parameter checker
- `simulation/webots/`: worlds, protos, and controllers
- `stm32_fw/hat_v1/`: firmware source, protocol docs, flashing/integration runbooks
- `scripts/`: setup, install, deployment, and utility scripts
- `rules.md`: CoVAPSy rule summary and constraints reference

## Prerequisites

Recommended host environment:

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12
- Colcon + rosdep
- Webots R2025a (for simulation)

Typical install baseline (host):

```bash
sudo apt update
sudo apt install -y \
	python3-colcon-common-extensions \
	python3-rosdep \
	python3-pip
```

Then ensure ROS2 is sourced:

```bash
source /opt/ros/jazzy/setup.bash
```

## Quick Start (ROS2 Workspace)

Build from source:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Verify package executables:

```bash
ros2 pkg executables covapsy_lidar
ros2 pkg executables covapsy_control
ros2 pkg executables covapsy_bridge
ros2 pkg executables covapsy_direction
ros2 pkg executables covapsy_learning
ros2 pkg executables covapsy_tools
```

## How To Run

### Launch Modes

From `ros2_ws/` after sourcing:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Simulation-oriented launches:

```bash
ros2 launch covapsy_bringup sim_webots.launch.py mode:=race
ros2 launch covapsy_bringup sim_webots.launch.py mode:=learning
ros2 launch covapsy_bringup sim_webots.launch.py mode:=reactive
```

Legacy ROS-only simulation launches (keep Webots lifecycle external):

```bash
ros2 launch covapsy_bringup sim_reactive.launch.py
ros2 launch covapsy_bringup sim_learning.launch.py
ros2 launch covapsy_bringup sim_race.launch.py
```

Real-car-oriented launches:

```bash
ros2 launch covapsy_bringup car_safe.launch.py
ros2 launch covapsy_bringup car_learning.launch.py
ros2 launch covapsy_bringup car_race.launch.py
```

### Basic Runtime Observability

In another terminal (same sourcing):

```bash
ros2 topic echo /car_mode
ros2 topic echo /race_telemetry
ros2 topic echo /recovery_state
ros2 topic echo /recovery_debug
```

### Manual Trigger Tests

Wrong-way trigger:

```bash
ros2 topic pub /wrong_direction std_msgs/msg/Bool "{data: true}" -r 10
ros2 topic pub /wrong_direction_confidence std_msgs/msg/Float32 "{data: 0.95}" -r 10
```

Rear-obstacle injection:

```bash
ros2 topic pub /rear_obstacle std_msgs/msg/Bool "{data: true}" -r 10
```

## How It Works (Node-by-Node)

### `covapsy_lidar`

- `scan_filter_node`
	- Subscribes: `/scan`
	- Publishes: `/scan_filtered`
	- Function: clamps invalid ranges and applies circular median filtering.

- `reactive_driver_node`
	- Subscribes: `/scan_filtered`
	- Publishes: `/cmd_drive_reactive`, `/reactive_debug`
	- Function:
		- forward FOV extraction
		- nearest-point safety bubble
		- disparity extension using car footprint
		- largest-gap target selection
		- steering slew limiting
		- speed policy based on clearance, curvature, and TTC

### `covapsy_direction`

- `direction_detector_node`
	- Subscribes: `/image_raw`, `/scan_filtered`
	- Publishes: `/wrong_direction`, `/wrong_direction_confidence`, `/track_direction`
	- Function: color-based direction estimation (green/red borders) with LiDAR-gated ROIs and hysteresis for wrong-way latching.

### `covapsy_learning`

- `track_learner_node`
	- Subscribes: `/scan_filtered`, `/odom`, `/track_direction`
	- Publishes: `/track_path`, `/track_quality`, `/track_learned`
	- Function:
		- samples centerline estimates from odom + side clearances
		- resamples and smooths trajectory
		- computes quality metrics (closure, spacing, smoothness, consistency)
		- validates against minimum quality threshold

### `covapsy_control`

- `pure_pursuit_node`
	- Subscribes: `/track_path`, `/odom`, `/scan_filtered`
	- Publishes: `/cmd_drive_pursuit`
	- Function: geometric pure pursuit with dynamic lookahead, steering slew limit, and clearance/curvature-aware speed scheduling.

- `mode_controller_node`
	- Subscribes:
		- `/cmd_drive_reactive`, `/cmd_drive_pursuit`
		- `/wrong_direction`, `/wrong_direction_confidence`
		- `/track_learned`, `/track_quality`
		- `/scan_filtered`, `/wheel_speed`, `/rear_obstacle`, `/imu/data`
	- Publishes: `/cmd_drive`, `/car_mode`, `/recovery_state`, `/recovery_debug`, `/race_telemetry`
	- Function:
		- mode transitions (`IDLE`, `LEARN`, `RACE`, `RECOVERY`, `STOPPED`)
		- LEARN: reactive primary with speed cap
		- RACE: pursuit nominal + reactive safety overlay
		- RECOVERY: bounded FSM (BRAKE -> REVERSE -> REASSESS -> ESCALATE -> FAILSAFE_STOP)
		- wrong-way/no-progress/wedge trigger detection

### `covapsy_bridge`

- `stm32_bridge_node`
	- Subscribes: `/cmd_drive`, `/race_start`, `/race_stop`
	- Publishes: `/wheel_speed`, `/rear_obstacle`, `/bridge_status`, `/stm32/cmd_drive`
	- Function:
		- enforces start/stop gating
		- watchdog timeout failsafe to neutral/brake
		- speed/steer clamping before low-level output

### `covapsy_tools`

- `telemetry_logger_node`: writes `/race_telemetry` to CSV.
- `param_sanity_checker`: quick parameter query sanity check.

## Configuration Profiles

The launch files load YAML profiles in `covapsy_bringup/config/`:

- `learn_sim.yaml`: conservative learning profile for simulation.
- `race_sim.yaml`: race profile for simulation.
- `learn_real.yaml`: conservative real-car learning profile with stricter bridge safety defaults.
- `race_real.yaml`: real-car race profile with competition gating defaults.

Important bridge defaults in real-car profiles include:

- `competition_mode: true`
- `require_start_signal: true`
- `allow_restart_after_stop: false`
- tight watchdog timeout

## Simulation

Webots assets are in `simulation/webots/`.

Worlds include:

- `Piste_CoVAPSy_2025a_ros2.wbt`
- `Piste_CoVAPSy_2025a_multicar_ros2.wbt`
- `Piste_CoVAPSy_2025a_standalone.wbt`
- `Piste_CoVAPSy_2025a_multicar_standalone.wbt`

Controllers include:

- `controllers/covapsy_ros2_bridge/covapsy_ros2_bridge.py` (ROS2 bridge mode)
- `controllers/covapsy_standalone/covapsy_standalone.py` (standalone mode)

Unified launch (starts Webots and ROS stack together):

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch covapsy_bringup sim_webots.launch.py mode:=race
```

Useful launch arguments:

```bash
# Choose world
ros2 launch covapsy_bringup sim_webots.launch.py \
	mode:=race world:=Piste_CoVAPSy_2025a_multicar_ros2.wbt

# Headless Webots
ros2 launch covapsy_bringup sim_webots.launch.py mode:=race headless:=true
```

Manual Webots start (if using legacy ROS-only sim launch files):

```bash
webots simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

Windows helper script (standalone world):

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_webots_standalone.ps1
```

## STM32 Firmware

Firmware workspace: `stm32_fw/hat_v1/`

What is included:

- protocol and control mapping source (`src/`)
- CubeIDE project scaffold (`cubeide_project/`)
- integration docs (`docs/`)
- release notes (`releases/`)

Key behaviors documented in firmware workspace:

- SPI slave 6-byte frame protocol (`0x55 0x55` header)
- watchdog neutral after timeout
- steering/speed PWM mapping
- telemetry frame (`wheel_speed`, `rear_obstacle`)

Start with:

- `stm32_fw/hat_v1/docs/pinmap.md`
- `stm32_fw/hat_v1/docs/windows_cubeide_setup.md`
- `stm32_fw/hat_v1/docs/flashing_swd_dfu.md`
- `stm32_fw/hat_v1/docs/bench_validation.md`
- `stm32_fw/hat_v1/docs/pi_integration.md`

## Deployment to Raspberry Pi

Host -> Pi source sync:

```bash
bash scripts/deploy_to_pi.sh <pi_ip_or_hostname>
```

Pi bootstrap:

```bash
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

Build on Pi:

```bash
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

## Debugging and Validation

Useful checks:

```bash
ros2 topic list
ros2 topic hz /scan_filtered
ros2 topic hz /cmd_drive
ros2 topic echo /bridge_status
ros2 topic echo /race_telemetry
```

Recovery-focused checks:

```bash
ros2 topic echo /recovery_state
ros2 topic echo /recovery_debug
```

Log telemetry:

```bash
ros2 run covapsy_tools telemetry_logger_node --ros-args -p csv_path:=/tmp/covapsy_telemetry.csv
```

Path utility from setup-lap CSV:

```bash
python3 scripts/build_racing_path_from_csv.py \
	--input setup_lap.csv \
	--output racing_path.json
```

## Known Integration Notes

1. The ROS2 autonomy stack in `ros2_ws/src/` exchanges `covapsy_interfaces/msg/DriveCommand`.
2. Some Webots bridge/controller paths use `/cmd_vel` (`geometry_msgs/msg/Twist`).
3. If you combine those components directly, add or enable an explicit command adapter (`DriveCommand <-> Twist`) so command topics/types are consistent.
4. The setup scripts and docs include both baseline and legacy references; use launch files present in `covapsy_bringup/launch/` as ground truth for this workspace state.

## Extra Docs

- `ros2_ws/README.md`: ROS2 baseline stack summary
- `ros2_ws/RUNBOOK.md`: execution checklist and scenario validation
- `rules.md`: CoVAPSy rules summary and constraints
