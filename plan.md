# COVAPSY Autonomous Racing Guide for Raspberry Pi 5 (2GB)

The COVAPSY competition at ENS Paris-Saclay runs 1/10-scale Tamiya TT-02 autonomous cars on unknown tracks. The 6th edition is scheduled for April 11, 2026. This guide defines one canonical baseline: Raspberry Pi 5 (2GB) for high-level autonomy, STM32 for deterministic motor control, and LiDAR-first perception with camera as optional augmentation. The target is race reliability first, then lap-time optimization.

## Competition constraints that should drive every design choice

COVAPSY constraints define the architecture more than algorithm preference:

- Chassis: Tamiya TT-02 mandatory.
- Battery: NiMH 7.2V (5000 mAh max, per competition ruleset).
- Track geometry: unknown before race day, no preloaded track map allowed.
- Borders: right border green, left border red, minimum turn radius 400 mm, width above 800 mm.
- Control during race: remote start/stop only. Behavior-changing teleoperation is disallowed.
- Homologation expectations: straight + turn handling, reverse behavior when blocked, obstacle handling, and safe stop behavior.
- 2026 rule extension: trackside compute is allowed (off-car compute can run heavy algorithms if untouched after start).

The practical implication is dual-mode autonomy:

1. Reactive mode for immediate deployment on unknown layout.
2. Map-aware mode after setup laps if localization remains stable.

## Canonical hardware baseline for Raspberry Pi 5 (2GB)

### Pi 5 platform characteristics

| Item | Raspberry Pi 5 (2GB) baseline | Design impact |
|---|---|---|
| CPU | 4x Cortex-A76 @ 2.4 GHz | Enough for ROS2 Jazzy + SLAM + planning with tight memory discipline |
| RAM | 2GB LPDDR4X | Main system constraint, requires explicit workload budgeting |
| USB | USB 3.0 + USB 2.0 via RP1 | Better LiDAR and peripheral stability than older shared-bus designs |
| PCIe | PCIe 2.0 x1 (Gen 3 optional) | Enables NVMe swap and optional AI accelerator |
| UART | Multiple PL011 UARTs | Cleaner STM32 communication options |
| Camera I/O | 22-pin MIPI connectors | Requires adapter cables for older camera ribbons |
| Power expectation | 5V/5A recommended | Strongly affects regulator and wiring design |
| Thermals | Active cooling required | Prevents sustained-load throttling |

### Reference car architecture

| Component | Reference choice | Role |
|---|---|---|
| Chassis | Tamiya TT-02 | Mechanical base and drivetrain |
| Main compute | Raspberry Pi 5 (2GB) | Perception, mapping, planning, high-level decision logic |
| Microcontroller | STM32 NUCLEO-G431KB | 100-200 Hz control loop, PWM, encoder capture, watchdog fail-safe |
| LiDAR | SLAMTEC RPLidar A2M12 | 2D obstacle sensing, wall geometry, SLAM input |
| Camera | Raspberry Pi Camera module | Border color and lane-shape cues, optional detection tasks |
| IMU + encoder | Onboard stack | Heading and speed stabilization |
| Power conversion | 5V/5A buck converter to USB-C input path | Stable Pi power with proper boot behavior |
| Cooling | Active cooler | Sustained autonomy without thermal collapse |

## Mandatory bring-up prerequisites (power, cooling, firmware, and boot config)

These are non-optional for a stable Pi 5 race platform:

1. Power the board through USB-C path, not direct 5V GPIO injection.
2. Use a converter that can sustain 5V/5A peaks with margin.
3. Install active cooling and validate temperature under realistic load.
4. Set EEPROM and boot config for current limits and I/O behavior before autonomy testing.

### Boot and I/O configuration checklist

`/boot/firmware/config.txt` baseline:

```ini
# UART for STM32 link
dtparam=uart0=on

# Raise USB current budget when power path supports it
usb_max_current_enable=1

# Optional: higher PCIe bandwidth for NVMe/accelerator
dtparam=pciex1_gen=3
```

EEPROM setting:

```bash
sudo -E rpi-eeprom-config --edit
# Ensure this line exists:
# PSU_MAX_CURRENT=5000
```

Thermal validation target:

- Full-stack test load should stay below throttling thresholds during 10+ minute runs.
- Any sustained thermal throttling invalidates lap-time tuning data.

## Software stack: Ubuntu 24.04 + ROS2 Jazzy + CycloneDDS + micro-ROS

### OS and middleware baseline

- OS: Ubuntu Server 24.04 LTS (headless).
- ROS: ROS2 Jazzy (`ros-base` profile on-car).
- DDS: CycloneDDS (`rmw_cyclonedds_cpp`).
- MCU bridge: `micro_ros_agent` on Pi, micro-ROS client on STM32.

### Core installation profile

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rplidar-ros \
  ros-jazzy-micro-ros-agent \
  python3-colcon-common-extensions

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

Workspace flow:

```bash
mkdir -p ~/covapsy_ws/src
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo "source ~/covapsy_ws/install/setup.bash" >> ~/.bashrc
```

Micro-ROS agent example:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```

### Memory budget for 2GB baseline

| Layer | Typical RAM envelope |
|---|---|
| OS + background services | 250-350 MB |
| ROS graph + drivers | 180-300 MB |
| `slam_toolbox` (online async, tuned) | 350-550 MB |
| Nav2 (composed nodes) | 180-350 MB |
| Safety + margin | 250-400 MB |
| Total operational envelope | ~1.2-1.9 GB |

Operational policy for 2GB:

- Use composition where possible to reduce process overhead.
- Keep on-car GUI tools disabled.
- Use swap on NVMe if available, but do not treat swap as primary memory.
- Move heavy map optimization, training, and visualization to trackside compute.

## On-car versus trackside compute split

The 2026 rules make workload placement a strategic advantage:

| Workload | On-car Pi 5 (2GB) | Trackside compute |
|---|---|---|
| LiDAR driver + safety layer | Mandatory | Optional monitor only |
| STM32 bridge + command watchdog | Mandatory | Not applicable |
| Reactive avoidance (FTG/Disparity) | Mandatory | Not recommended as primary |
| SLAM map optimization | Optional light | Preferred |
| Global trajectory optimization | Optional coarse | Preferred |
| Visualization (RViz, dashboards) | Avoid | Preferred |
| Model training | Avoid | Mandatory |

Design rule: if losing WiFi would make the car unsafe, the workload is misplaced. Safety and minimum autonomous continuity must remain on-car.

## LiDAR pipeline for mapping and obstacle handling

### Recommended SLAM path on 2GB

- Default: `slam_toolbox` in online async mode with conservative map update settings.
- Use map building during setup laps at controlled speed, then lock into localization-centric operation for racing.
- Keep scan preprocessing simple and deterministic (range clipping + median or moving average).

Practical starting parameters:

- `max_laser_range`: 5.0 m
- `map_update_interval`: 0.2 s
- `solver_threads`: match available CPU while preserving planning headroom

### Reactive fallback: Follow-the-Gap + Disparity extension

Reactive control must always remain available as a fallback mode:

1. Filter scan noise.
2. Identify nearest obstacle.
3. Apply safety bubble.
4. Find largest feasible gap.
5. Pick steering target weighted by free-space depth.
6. Reduce speed with steering magnitude and closing-rate heuristics.

```python
def follow_the_gap(ranges, angles, car_radius, max_range):
    clipped = [min(max(r, 0.0), max_range) for r in ranges]
    nearest = min(range(len(clipped)), key=lambda i: clipped[i])
    blocked = clipped[:]
    for i in range(len(blocked)):
        if clipped[nearest] * abs(angles[i] - angles[nearest]) < car_radius:
            blocked[i] = 0.0
    gaps = consecutive_nonzero_indices(blocked)
    best = max(gaps, key=len) if gaps else [nearest]
    target = max(best, key=lambda i: blocked[i])
    return angles[target]
```

## Planning and control strategy under race conditions

### Two-mode control stack

1. Mode A (safe reactive):
- LiDAR-driven gap following.
- Lower speed ceiling.
- Wider clearance margins.

2. Mode B (mapped performance):
- Localized track following with precomputed trajectory.
- Pure Pursuit or Stanley tracking.
- Velocity profile constrained by curvature and braking envelope.

### Velocity profile essentials

At each waypoint:

- Curvature-limited speed: `v_max = sqrt(a_lat_max / abs(curvature))`
- Forward pass: acceleration constraints.
- Backward pass: braking constraints.
- Final speed: minimum of all constraints at each waypoint.

This yields a robust profile that can be adjusted globally with a race-condition multiplier.

### Controller choice guidance

- Pure Pursuit: best complexity/performance tradeoff for competition reliability.
- Stanley: stronger cross-track correction, more sensitive to noisy paths.
- MPC: optional if team has strong solver tuning discipline and explicit CPU budget.

## Camera pipeline and optional neural inference tiers

Camera usage should be workload-tiered to protect the 2GB memory budget.

### Tier 0 (recommended default): classical CV only

- Bird's-eye transform from calibrated mount.
- HSV masking for green and red borders.
- Morphology + contour or Hough extraction.
- Steering offset from lane-border geometry.

This tier is low-latency and leaves compute room for mapping and control.

### Tier 1 (conditional): lightweight neural detection on CPU

- Run compact models at reduced resolution and controlled rate.
- Trigger model execution only in uncertain scenes.
- Guard with watchdog timeout so control loop never blocks.

### Tier 2 (optional hardware): PCIe accelerator

- Use only if detection is a primary race objective.
- Keep autonomy functional without the accelerator to avoid single-point failure.

## STM32 real-time control and micro-ROS interface contract

The STM32 remains the timing authority for actuators.

### Control loop responsibilities (STM32)

- Servo PWM and ESC PWM generation.
- Encoder acquisition and speed estimation.
- PID speed loop with anti-windup.
- Independent watchdog and command timeout fail-safe.

### ROS2 topic contract (example)

- Pi to STM32: `/cmd_drive` (steering, target speed, optional mode flags).
- STM32 to Pi: `/wheel_speed`, `/imu/data`, `/mcu_status`.
- Timeout policy: if command stream exceeds timeout window, force zero throttle and neutral steering.

Recommended transport:

- UART with DMA-backed buffering.
- Deterministic packet framing and CRC.
- micro-ROS on STM32, `micro_ros_agent` on Pi.

## Pi 5 integration details that commonly cause failures

### UART and GPIO

- Use hardware UART for STM32 link and reserve debug channels explicitly.
- Validate logic levels and ground integrity before software debugging.
- Keep actuator PWM generation on STM32 to avoid Linux scheduling jitter concerns.

### Camera connectors

- Pi 5 uses 22-pin connectors; verify cable type and orientation.
- Lock camera mount geometry. Recalibrate transform whenever mount angle changes.

### Power integrity

- Validate voltage sag during acceleration events, not just bench idle.
- Separate noisy motor return paths from compute power return where possible.
- Brownout events should be treated as hardware faults, not software bugs.

### Cooling and enclosure airflow

- Fan operation should be verified under real race vibration.
- Any closed enclosure requires explicit airflow path design.

## Simulation, testing, and acceptance gates

Simulation-first workflow:

1. Unit test perception and control components offline.
2. Integrate full graph in simulation.
3. Run speed ladder on real car: 30%, 60%, 90%, then endurance.
4. Record bags for every run and review failure cases before parameter changes.

Minimum acceptance gates before competition:

1. 20 consecutive clean laps in practice layout without border contact.
2. Verified recovery behavior for blocked path and obstacle insertion.
3. Command-loss fail-safe tested repeatedly.
4. Thermal stability under full mission duration.
5. Power stability with no brownout across battery discharge range.

## Recommended Stack and Race-Day Priorities

Canonical 2026 stack for this guide:

- Raspberry Pi 5 (2GB), Ubuntu Server 24.04, ROS2 Jazzy, CycloneDDS.
- LiDAR-first navigation with `slam_toolbox` + reactive fallback.
- micro-ROS bridge to STM32 for deterministic actuation and safety.
- Trackside computer for heavy optimization and visualization, not for safety-critical continuity.

Race-day priority order:

1. Safety and reliability: watchdogs, thermal headroom, clean power, predictable fallback mode.
2. Completion consistency: repeated clean laps beat unstable peak speed.
3. Speed optimization: trajectory and velocity profile tuning only after reliability gates pass.
4. Adaptability: keep a reactive mode ready for unknown layouts and race traffic.

This document intentionally defines one platform baseline only. All commands, architecture decisions, and validation gates here assume Raspberry Pi 5 (2GB).
