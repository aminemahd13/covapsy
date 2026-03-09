# Validation Protocol

Run this before track sessions and after major code changes.

## 1. Static Checks
```bash
python -m compileall mycar/ros2_ws/src mycar/simulation/webots/controllers
```

Expected:
- No syntax errors.

## 2. Unit Tests
```bash
cd mycar/ros2_ws
python -m pytest -q -p no:cacheprovider src/covapsy_perception/test src/covapsy_nav/test src/covapsy_bridge/test
```

Expected:
- All tests pass.

## 3. Simulation Integration Test
1. Terminal A:
```bash
cd mycar/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch covapsy_bringup sim_webots.launch.py
```
2. Terminal B (same sourced env):
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```
3. Terminal C checks:
```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel
ros2 topic echo /odom --once
ros2 topic echo /mcu_status --once
```

Pass criteria:
- all required topics present
- reactive command generated
- final `/cmd_vel` published continuously
- one clean autonomous lap in `REACTIVE` mode

## 4. Real-Car Smoke Test (Per Backend)
For each backend (`spi`, `uart`, `pi_pwm`):
1. Wheels off ground.
2. Launch:
```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=<backend>
```
3. Arm run (required in competition mode):
```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
```
4. Publish reactive command (mode controller forwards to bridge command topic):
```bash
ros2 topic pub /cmd_vel_reactive geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```
5. Stop publisher and verify watchdog stop within configured timeout.
6. Issue stop signal and verify immediate zero command:
```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```
7. Check status topic:
```bash
ros2 topic echo /mcu_status --once
```

Pass criteria:
- backend initializes without errors
- actuator response corresponds to command
- stale command triggers stop
- `/race_stop` immediately forces zero output
- status includes `race_running` and `stop_latched` fields

## 5. Safety Regression Checks
- Remote stop transitions to `STOPPED`.
- `IDLE` and `STOPPED` always publish zero on the configured command topic.
- Runtime `/set_mode` is ignored in competition bringup.
- LiDAR loss handling: mode controller should not command unsafe motion.
