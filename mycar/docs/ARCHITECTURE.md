# Architecture

## Topology
Simulation and real-car stacks share the same high-level ROS contracts.

Core runtime chain:
1. `/scan` -> `scan_filter_node` -> `/scan_filtered`
2. `/scan_filtered` -> `gap_follower_node` -> `/cmd_vel_reactive`
3. Optional `/racing_path` + `/odom` -> `pure_pursuit_node` -> `/cmd_vel_pursuit`
4. `mode_controller_node` selects final command on `/cmd_vel`
5. `/cmd_vel` -> `stm32_bridge_node` (real) or Webots bridge controller (sim)

## Topic Contract
| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_node` (real) or `covapsy_ros2_bridge` (sim) | `scan_filter_node` |
| `/scan_filtered` | `sensor_msgs/LaserScan` | `scan_filter_node` | `gap_follower_node`, `slam_toolbox` |
| `/cmd_vel_reactive` | `geometry_msgs/Twist` | `gap_follower_node` | `mode_controller_node` |
| `/cmd_vel_pursuit` | `geometry_msgs/Twist` | `pure_pursuit_node` | `mode_controller_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | `mode_controller_node` | bridge node / sim bridge |
| `/wheel_speed` | `std_msgs/Float32` | bridge node / sim bridge | `tft_status_node`, monitoring |
| `/rear_obstacle` | `std_msgs/Bool` | bridge node / sim bridge | `gap_follower_node` |
| `/mcu_status` | `std_msgs/String` | bridge node / sim bridge | monitoring |
| `/car_mode` | `std_msgs/String` | `mode_controller_node` | `tft_status_node`, monitoring |
| `/odom` | `nav_msgs/Odometry` | sim bridge (GPS/IMU) or localization (real) | `pure_pursuit_node` |

## Simulation Path
- Controller: `simulation/webots/controllers/covapsy_ros2_bridge`.
- World: `Piste_CoVAPSy_2025a_ros2.wbt` with one ROS2-driven `TT02_ros2`.
- Ground-truth odometry uses Webots `GPS` + `InertialUnit` sensors.
- Watchdog: if `/cmd_vel` is stale (`cmd_timeout_sec`), vehicle is commanded to stop.

## Real-Car Path
Bridge node: `covapsy_bridge/stm32_bridge_node.py`.

Backends:
- `spi`:
  - Pi is master (`spidev`), STM32 is slave.
  - Default frame length is 6 bytes.
  - Parameters define header bytes and index layout.
- `uart`:
  - Framed UART command path, telemetry parser for speed and rear obstacle.
- `pi_pwm`:
  - Direct PWM generation on Pi (`rpi-hardware-pwm`).
  - Uses calibrated duty-cycle mapping for propulsion and steering.

All backends expose the same ROS API (`/cmd_vel`, `/wheel_speed`, `/rear_obstacle`, `/mcu_status`).

## Safety and Watchdogs
- `mode_controller_node` enforces stop in `IDLE` and `STOPPED`.
- `stm32_bridge_node` watchdog sends zero command on stale `/cmd_vel`.
- Webots bridge controller applies equivalent stale-command stop behavior.

## Frames
- `base_link`: vehicle body reference.
- `laser`: LiDAR frame.
- `odom`: local odometry frame.
- Static transforms are started by bringup launch files.
