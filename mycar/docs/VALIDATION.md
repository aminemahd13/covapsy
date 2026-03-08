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
3. Publish command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```
4. Stop publisher and verify watchdog stop within configured timeout.
5. Check status topic:
```bash
ros2 topic echo /mcu_status --once
```

Pass criteria:
- backend initializes without errors
- actuator response corresponds to command
- stale command triggers stop

## 5. Safety Regression Checks
- Remote stop transitions to `STOPPED`.
- `IDLE` and `STOPPED` always publish zero `/cmd_vel`.
- LiDAR loss handling: mode controller should not command unsafe motion.
