# Calibration Workflow

Calibrate on bench first, then on track.

Safety:
- Wheels off ground for all first-time actuator checks.
- Keep remote stop available.
- Start with `car_safe.launch.py`.

## 1. Steering Calibration (`pi_pwm` backend)
Edit in `backend_params.yaml`:
- `pwm_direction_sign`
- `pwm_angle_min`
- `pwm_angle_max`
- `pwm_angle_center`
- `pwm_angle_deg_max`

Professor baseline values from `test_pwm_direction.py`:
- `pwm_direction_sign = -1`
- `pwm_angle_min = 5.5`
- `pwm_angle_max = 9.3`
- `pwm_angle_center = 7.4`
- `pwm_angle_deg_max = 18`

Procedure:
1. Launch:
```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=pi_pwm initial_mode:=REACTIVE
```
2. Publish test commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}" -r 5
```
3. Confirm steering direction and centered straight at zero command.
4. Reduce min/max if servo hits mechanical stops.

## 2. Propulsion Calibration (`pi_pwm` backend)
Edit:
- `pwm_prop_stop`
- `pwm_prop_deadband`
- `pwm_prop_delta_max`
- `pwm_speed_hard`
- `max_speed_fwd`, `max_speed_rev`

Professor baseline values from `test_pwm_propulsion.py`:
- `pwm_prop_stop = 7.5`
- `pwm_prop_deadband = 0.56`
- `pwm_prop_delta_max = 1.0`
- `pwm_speed_hard = 8.0`

Current repository defaults in `backend_params.yaml`:
- `pwm_prop_stop = 7.5`
- `pwm_prop_deadband = 0.56`
- `pwm_prop_delta_max = 1.0`
- `pwm_speed_hard = 8.0`

Procedure:
1. Start with low command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 5
```
2. Increase deadband if motor does not move.
3. Decrease deadband or max delta if startup is too aggressive.
4. Verify zero command returns full stop.

## 3. SPI/UART Command Path Calibration
For `spi` or `uart` backends:
- Keep PWM source jumpers on STM32 side.
- Tune limits in `backend_params.yaml`:
  - `max_steering_deg`
  - `max_speed_fwd`
  - `max_speed_rev`
- For SPI framing mismatches, adjust:
  - `spi_header_0`, `spi_header_1`
  - byte indexes (`spi_*_index`)

## 4. Watchdog Validation
1. Publish non-zero command at 10 Hz.
2. Stop publisher.
3. Confirm stop within configured timeout (`watchdog_timeout`, default 0.25 s).
4. Check status:
```bash
ros2 topic echo /mcu_status --once
```

## 5. End-to-End Sanity
```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel
```

Expected:
- Stable LiDAR pipeline.
- Non-zero reactive command on open path.
- Final `/cmd_vel` consistent with `mode_controller` state.
