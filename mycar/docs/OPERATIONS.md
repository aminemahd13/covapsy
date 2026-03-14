# Operations Runbook (Learning -> Racing with Direction-Aware Persistence)

This runbook explains how the stack now performs:
- real-time learning during setup laps,
- direction-aware path persistence,
- automatic saved-path reuse on later runs,
- fallback to live learning when no saved path exists.

## 1. Competition Contract (What Is Allowed)
- After race start, team-to-car commands must be limited to start/stop.
- No runtime human behavior commands (mode forcing, steering commands, etc.) in competition runs.
- Reverse capability must work.
- Avoid prolonged immobilization and wrong-way driving.

Current stack defaults are aligned to this:
- runtime `/set_mode` switching is disabled in competition bringups (`allow_runtime_mode_switch:=false`)
- start/stop topics are used as control contract
- learning-to-racing handoff is automatic once track learning is confirmed
- saved-path reuse is automatic and direction-aware (`/track_direction`)

## 2. State Machine and Transition Logic
`mode_controller_node` publishes `/car_mode` and selects the final command stream.

Modes:
- `IDLE`: no motion output
- `LEARNING`: reactive driving with learning speed cap
- `RACING`: pure pursuit (or tactical when enabled and valid)
- `STOPPED`: latched stop
- recovery states are published as `REVERSING` or `UTURN` when active

Automatic handoff and reuse:
1. `border_detect_node` infers direction from border colors and publishes `/track_direction` (`red_left`, `red_right`, `unknown`).
2. `racing_path_publisher_node` tries to load `~/.ros/covapsy/racing_path_<direction>.json`.
3. If found, it publishes `/racing_path`, `/saved_track_loaded=true`, and `/track_learned=true`.
4. If missing, it publishes `/saved_track_loaded=false`; `track_learner_node` continues live learning.
5. During fallback learning, after `required_laps`, `track_learner_node` publishes `/racing_path`, `/track_learned=true`, and saves to disk.
6. `mode_controller_node` waits `track_learned_handoff_confirm_sec` then auto-switches from `LEARNING` to `RACING`.

Default persisted files:
- `~/.ros/covapsy/racing_path_red_left.json`
- `~/.ros/covapsy/racing_path_red_right.json`
- `~/.ros/covapsy/racing_path_unknown.json`

## 3. Simulation (Webots + ROS2)

### 3.1 Default behavior in `sim_webots.launch.py`
- `initial_mode:=LEARNING`
- `start_mode:=LEARNING`
- `enable_track_learning:=true`
- `enable_saved_track_reuse:=true`
- `enable_border_detect:=true`
- `enable_pure_pursuit:=true`
- `track_learning_required_laps:=1`
- `track_learned_handoff_confirm_sec:=0.60`
- `track_storage_dir:=~/.ros/covapsy`

Because initial mode is already active (`LEARNING`), simulation starts learning immediately after launch.

### 3.2 Run sequence
Terminal A:
```bash
cd mycar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch covapsy_bringup sim_webots.launch.py
```

Terminal B (same sourced env):
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

Terminal C (monitor):
```bash
ros2 topic echo /car_mode
ros2 topic echo /track_learned
ros2 topic echo /saved_track_loaded
ros2 topic echo /track_direction
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel_pursuit
```

Expected transition:
- startup: `/car_mode` is `LEARNING`
- if a saved directional path exists: `/saved_track_loaded` is `true` and handoff to `RACING` happens quickly
- if no saved directional path exists: `/saved_track_loaded` stays `false`, learning runs, then handoff occurs after required lap(s)

### 3.3 Stop/restart cycle in simulation
Stop run:
```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```

Notes:
- Stop is latched in competition-style defaults; restart ROS launch + Webots world for a new run.
- Wrong-way telemetry in sim is available on `/wrong_direction` and `/wrong_direction_confidence`.
- Direction telemetry is available on `/track_direction`.

### 3.4 Multi-car simulation
```bash
ros2 launch covapsy_bringup sim_webots_multicar.launch.py
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_multicar_ros2.wbt
```
- Tactical AI is enabled by default in this launch and context-gated (`tactical_context_mode:=auto`).

## 4. Real Car (Pi + LiDAR + Bridge)

### 4.1 Default behavior in `car_full.launch.py`
- `initial_mode:=IDLE`
- `start_mode:=LEARNING`
- `enable_track_learning:=true`
- `enable_saved_track_reuse:=true`
- `enable_border_detect:=true`
- `enable_pure_pursuit:=true`
- `track_learning_required_laps:=2`
- `track_learned_handoff_confirm_sec:=0.80`
- `track_storage_dir:=~/.ros/covapsy`
- command output topic: `/cmd_vel_autonomy`
- runtime `/set_mode`: disabled

This means the car waits in `IDLE` until a start signal is sent.

### 4.2 Recommended real-car sequence
1. Bench safety first:
```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=spi
```

2. Full run:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE start_mode:=LEARNING
```

3. Start learning laps:
```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
```

4. Monitor transition:
```bash
ros2 topic echo /car_mode
ros2 topic echo /track_learned
ros2 topic echo /saved_track_loaded
ros2 topic echo /track_direction
ros2 topic echo /mcu_status
```

Expected transition:
- pre-start: `IDLE`
- after start: `LEARNING`
- if directional path exists: quick handoff to `RACING` using saved path
- if directional path is missing: live setup-lap learning then handoff to `RACING`

5. Stop:
```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```

### 4.3 Re-running learning vs race on real car
- New full session: restart launch, then send `/race_start`.
- Keep learning enabled for unknown tracks (competition-safe default).
- Keep saved-path reuse enabled for race-day flow:
  - `enable_saved_track_reuse:=true`
  - `track_storage_dir:=~/.ros/covapsy`
- Solo runs will overwrite the direction-specific saved path; later runs auto-reuse it.
- Manual `path_file` injection remains available for debug/offboard workflows only.

Optional manual override path publisher:
```bash
ros2 run covapsy_nav racing_path_publisher_node --ros-args -p path_file:=racing_path.json
```

## 5. How To Force Modes (Debug Only, Non-Competition)
For development only:
1. Launch with `allow_runtime_mode_switch:=true`.
2. Then publish:
```bash
ros2 topic pub /set_mode std_msgs/msg/String "{data: 'LEARNING'}" --once
ros2 topic pub /set_mode std_msgs/msg/String "{data: 'RACING'}" --once
```

Do not use this during competition runs.

## 6. Quick Troubleshooting for Learning -> Racing
- Stays in `LEARNING`:
  - check `/track_learned` topic
  - check `/saved_track_loaded` and `/track_direction`
  - verify odometry is present (`/odom`)
  - verify `enable_track_learning:=true`
  - verify required laps is achievable for the current track
  - verify saved files exist under `~/.ros/covapsy` for the current direction
- Starts in wrong mode:
  - check launch args `initial_mode` and `start_mode`
- Car not moving on real car:
  - verify `/race_start` was sent
  - verify `/car_mode` is not `IDLE` or `STOPPED`
  - verify `/mcu_status` reports backend OK

## 7. Launch Recipes (Common Switching Patterns)
Simulation, strict setup-lap flow (wait for start signal first):
```bash
ros2 launch covapsy_bringup sim_webots.launch.py initial_mode:=IDLE start_mode:=LEARNING
```

Simulation, auto-reuse saved directional path (default persistence flow):
```bash
ros2 launch covapsy_bringup sim_webots.launch.py
```

Real car, auto-reuse saved directional path (default persistence flow):
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE start_mode:=LEARNING
```

Manual race-only debug with explicit prebuilt path:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE start_mode:=RACING enable_track_learning:=false enable_saved_track_reuse:=false
ros2 run covapsy_nav racing_path_publisher_node --ros-args -p path_file:=racing_path.json
```
