# Operations Runbook (Learning -> Racing)

This runbook explains exactly how to run setup/learning laps and transition to race driving in simulation and on the real car.

## 1. Competition Contract (What Is Allowed)
- After race start, team-to-car commands must be limited to start/stop.
- No runtime human behavior commands (mode forcing, steering commands, etc.) in competition runs.
- Reverse capability must work.
- Avoid prolonged immobilization and wrong-way driving.

Current stack defaults are aligned to this:
- runtime `/set_mode` switching is disabled in competition bringups (`allow_runtime_mode_switch:=false`)
- start/stop topics are used as control contract
- learning-to-racing handoff is automatic once track learning is confirmed

## 2. State Machine and Transition Logic
`mode_controller_node` publishes `/car_mode` and selects the final command stream.

Modes:
- `IDLE`: no motion output
- `LEARNING`: reactive driving with learning speed cap
- `RACING`: pure pursuit (or tactical when enabled and valid)
- `STOPPED`: latched stop
- recovery states are published as `REVERSING` or `UTURN` when active

Automatic handoff:
1. `track_learner_node` records odometry and detects lap closure.
2. After `required_laps`, it publishes `/racing_path` and `/track_learned=true`.
3. `mode_controller_node` waits `track_learned_handoff_confirm_sec`.
4. Controller auto-switches from `LEARNING` to `RACING`.

## 3. Simulation (Webots + ROS2)

### 3.1 Default behavior in `sim_webots.launch.py`
- `initial_mode:=LEARNING`
- `start_mode:=LEARNING`
- `enable_track_learning:=true`
- `enable_pure_pursuit:=true`
- `track_learning_required_laps:=1`
- `track_learned_handoff_confirm_sec:=0.60`

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
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel_pursuit
```

Expected transition:
- startup: `/car_mode` is `LEARNING`
- after required lap(s): `/track_learned` becomes `true`
- handoff: `/car_mode` becomes `RACING`

### 3.3 Stop/restart cycle in simulation
Stop run:
```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```

Notes:
- Stop is latched in competition-style defaults; restart ROS launch + Webots world for a new run.
- Wrong-way telemetry in sim is available on `/wrong_direction` and `/wrong_direction_confidence`.

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
- `enable_pure_pursuit:=true`
- `track_learning_required_laps:=2`
- `track_learned_handoff_confirm_sec:=0.80`
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
ros2 topic echo /mcu_status
```

Expected transition:
- pre-start: `IDLE`
- after start: `LEARNING`
- after required setup laps and confirmation: `RACING`

5. Stop:
```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```

### 4.3 Re-running learning vs race on real car
- New full session: restart launch, then send `/race_start`.
- Keep learning enabled for unknown tracks (competition-safe default).
- If you must skip learning in controlled testing, launch with:
  - `enable_track_learning:=false`
  - `start_mode:=RACING`
  - and provide `/racing_path` from a trusted source

Example path publisher:
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
  - verify odometry is present (`/odom`)
  - verify `enable_track_learning:=true`
  - verify required laps is achievable for the current track
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

Simulation, race-only test with prebuilt path:
```bash
ros2 launch covapsy_bringup sim_webots.launch.py initial_mode:=RACING enable_track_learning:=false
ros2 run covapsy_nav racing_path_publisher_node --ros-args -p path_file:=racing_path.json
```

Real car, race-only test with prebuilt path:
```bash
ros2 launch covapsy_bringup car_full.launch.py backend:=spi initial_mode:=IDLE start_mode:=RACING enable_track_learning:=false
ros2 run covapsy_nav racing_path_publisher_node --ros-args -p path_file:=racing_path.json
```
