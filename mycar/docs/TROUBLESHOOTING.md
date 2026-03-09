# Troubleshooting

## Simulation Issues
### No `/scan` in simulation
Check:
1. World is `simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt`.
2. Car `TT02_ros2` controller is `covapsy_ros2_bridge`.
3. Webots started from a terminal where ROS2 environment is sourced.
4. `ros2 topic list` includes `/scan` and `/mcu_status`.

### Webots controller reports ROS import errors
Cause:
- Webots Python process cannot find ROS2 Python packages.

Fix:
- Start Webots from sourced shell:
```bash
source /opt/ros/jazzy/setup.bash
source mycar/ros2_ws/install/setup.bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

## Real-Car Issues
### No `/scan`
1. Verify LiDAR spins.
2. Verify device path (`/dev/rplidar`).
3. Launch LiDAR only:
```bash
ros2 launch covapsy_bringup lidar.launch.py
```

### Car does not move
1. Confirm race has been started and not stopped:
```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
ros2 topic echo /mcu_status --once
```
2. Confirm mode is not `IDLE` or `STOPPED`:
```bash
ros2 topic echo /car_mode --once
```
3. Confirm command exists on bridge input topic:
```bash
ros2 topic echo /cmd_vel_autonomy --once
```
4. Check backend status:
```bash
ros2 topic echo /mcu_status --once
```
5. Verify jumper routing matches backend selection.

### `spi` backend unstable or no response
1. Ensure `dtparam=spi=on`.
2. Verify `spi_bus`/`spi_device`.
3. Reduce `spi_speed_hz` to `500000`.
4. Check frame headers:
   - guide examples use `0x55 0x55`
   - adjust `spi_header_0`, `spi_header_1` if firmware differs

### `uart` backend no telemetry
1. Ensure `dtparam=uart0=on`.
2. Verify TX/RX wiring orientation.
3. Match baudrate with STM32 firmware (`uart_baudrate`).
4. Validate raw bytes with serial monitor if needed.

### `pi_pwm` backend wrong steering/propulsion direction
1. Check jumpers are on Pi source side.
2. Flip `pwm_direction_sign`.
3. Recheck `pwm_angle_*` and propulsion calibration values.

## Behavior Tuning Issues
### Wall clipping
- Increase `safety_radius`.
- Lower `max_speed`.
- Lower `disparity_threshold`.

### Oscillation
- Increase `lookahead_min` (if using pure pursuit).
- Reduce steering gain or max speed.
- Validate LiDAR filtering (`median_window`).

## Permission/Workspace Test Quirk
If pytest cache permission warnings appear on this machine, run:
```bash
python -m pytest -q -p no:cacheprovider
```
