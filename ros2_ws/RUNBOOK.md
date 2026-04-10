# Runbook

## Environment

- Ubuntu 24.04
- ROS2 Jazzy
- Webots R2025a

## Build

```bash
cd ~/Desktop/covapsy/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch Modes

```bash
ros2 launch covapsy_bringup sim_webots.launch.py mode:=race
ros2 launch covapsy_bringup sim_webots.launch.py mode:=learning
ros2 launch covapsy_bringup sim_webots.launch.py mode:=reactive
ros2 launch covapsy_bringup sim_reactive.launch.py
ros2 launch covapsy_bringup sim_learning.launch.py
ros2 launch covapsy_bringup sim_race.launch.py
ros2 launch covapsy_bringup car_safe.launch.py
ros2 launch covapsy_bringup car_learning.launch.py
ros2 launch covapsy_bringup car_race.launch.py
```

Note: `sim_webots.launch.py mode:=race` auto-selects `Piste_CoVAPSy_2025a_multicar_ros2.wbt` unless `world:=...` is provided.

## STM32 USB Link Checks (Real Car)

Before launching real-car profiles, confirm the USB symlink exists:

```bash
ls -l /dev/stm32_mcu
```

Bridge status values:

- `USB_DISCONNECTED`: serial device missing or cannot be opened
- `USB_TIMEOUT`: no telemetry received within timeout
- `USB_PARSE_ERROR`: malformed telemetry received
- `WAIT_START`, `WATCHDOG_BRAKE`, `RUN`: normal runtime states once USB link is healthy

## sim_reactive Validation Commands

```bash
cd ~/Desktop/covapsy/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch covapsy_bringup sim_reactive.launch.py
```

In another terminal:

```bash
cd ~/Desktop/covapsy/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /car_mode
ros2 topic echo /recovery_state
ros2 topic echo /recovery_debug
```

Optional trigger helpers:

```bash
# Rear-blocked recovery injection
ros2 topic pub /rear_obstacle std_msgs/msg/Bool "{data: true}" -r 10

# Wrong-direction trigger injection
ros2 topic pub /wrong_direction std_msgs/msg/Bool "{data: true}" -r 10
ros2 topic pub /wrong_direction_confidence std_msgs/msg/Float32 "{data: 0.95}" -r 10
```

## Testing Checklist

1. Plumbing only
- Launch sim_reactive.
- Verify /scan, /scan_filtered, /cmd_drive_reactive, /cmd_drive.

2. Reactive-only
- Keep mode in LEARN.
- Ensure multiple laps without repeated wall contacts.

3. Hard corners and wedge
- Force tight corner and wall wedge.
- Verify bounded recovery sequence and no infinite loops.

4. Direction tests
- Run both track directions in simulation.
- Check /wrong_direction and confidence hysteresis behavior.

5. Learning tests
- 1 lap then 2 laps.
- Validate closure error and smoothness from /track_quality.

6. Learning to race handoff
- Verify mode only switches LEARN -> RACE when /track_learned true and quality threshold met.

7. Race mode
- Confirm nominal path tracking by /cmd_drive_pursuit.
- Confirm reactive layer only caps speed, vetoes unsafe steering, or brakes.

8. Homologation-style checks
- Remote start/stop via /race_start and /race_stop.
- Straight + turn without touching.
- Reverse when blocked.
- Avoid car-sized obstacle.

## Recovery Scenarios (sim_reactive)

1. Straight corridor
- Expected trigger conditions: none.
- Expected mode transitions: LEARN only.
- Expected final outcome: continuous forward motion, no RECOVERY entry.

2. Simple turn
- Expected trigger conditions: none if clearance remains above blocked threshold.
- Expected mode transitions: LEARN only.
- Expected final outcome: car tracks turn without sustained stuck detection.

3. Chicane
- Expected trigger conditions: possible brief front-clearance drop but no persistent no-progress.
- Expected mode transitions: LEARN, optional short RECOVERY then back to LEARN.
- Expected final outcome: bounded correction, no infinite recovery loop.

4. Hairpin
- Expected trigger conditions: front clearance low with wedge signature or no-progress persistence.
- Expected mode transitions: LEARN -> RECOVERY -> ESCALATE -> LEARN.
- Expected final outcome: at least one bounded escape attempt, car retries forward.

5. Wedge after turn (front-left or front-right contact)
- Expected trigger conditions: commanded forward speed positive, wheel speed near zero, front blocked, side asymmetry.
- Expected mode transitions: LEARN -> RECOVERY(BRAKE->REVERSE->REASSESS->ESCALATE) -> LEARN.
- Expected final outcome: bounded escape attempt(s), no stuck-forever behavior.

6. Rear-blocked recovery
- Expected trigger conditions: RECOVERY reaches REVERSE while /rear_obstacle is true.
- Expected mode transitions: LEARN -> RECOVERY; on reverse block: BRAKE -> REASSESS -> (forward reorient if safe) -> ESCALATE -> LEARN.
- Expected final outcome: no immediate STOPPED on first blocked reverse; STOPPED only on repeated failed attempts or boxed front+rear.

7. Wrong-direction trigger
- Expected trigger conditions: /wrong_direction true with confidence above threshold.
- Expected mode transitions: LEARN -> RECOVERY -> ESCALATE -> LEARN (or STOPPED only if failures exceed limit).
- Expected final outcome: recovery action starts without tactical/camera steering authority changes.

## Recovery Debug Fields

The `/recovery_debug` topic publishes JSON with:
- trigger_reason
- recovery_state
- recovery_state_name
- reverse_blocked
- front_clearance_m
- left_clearance_m
- right_clearance_m
- attempt_count
