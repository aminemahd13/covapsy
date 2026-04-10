# CoVAPSy 2026 Baseline Stack (Clean Rebuild)

This workspace is a clean ROS2 Jazzy rebuild for CoVAPSy 2026 with the following authority model:

- LEARN mode: LiDAR reactive is primary.
- RACE mode: Pure Pursuit is primary.
- Reactive safety overlay: race-only veto/speed-cap/brake layer.
- RGB/RGB-D: direction and wrong-way confirmation only.
- Recovery FSM: bounded BRAKE -> REVERSE -> REASSESS -> ESCALATE -> FAILSAFE_STOP.
- Real-car split authority:
  - STM32 bridge: propulsion + telemetry + race safety watchdog.
  - DYNAMIXEL steering bridge: XL430 steering over USB (`/dev/waveshare_servo`).

## Package and File Tree

```
ros2_ws/
  README.md
  RUNBOOK.md
  src/
    covapsy_bringup/
      package.xml
      setup.py
      launch/
        sim_reactive.launch.py
        sim_learning.launch.py
        sim_race.launch.py
        car_safe.launch.py
        car_learning.launch.py
        car_race.launch.py
      config/
        learn_sim.yaml
        race_sim.yaml
        learn_real.yaml
        race_real.yaml
    covapsy_interfaces/
      package.xml
      CMakeLists.txt
      msg/
        TrackQuality.msg
        DirectionState.msg
        RecoveryState.msg
        RaceTelemetry.msg
    covapsy_lidar/
      package.xml
      setup.py
      covapsy_lidar/
        scan_filter_node.py
        reactive_driver_node.py
    covapsy_direction/
      package.xml
      setup.py
      covapsy_direction/
        direction_detector_node.py
    covapsy_learning/
      package.xml
      setup.py
      covapsy_learning/
        track_learner_node.py
    covapsy_control/
      package.xml
      setup.py
      covapsy_control/
        pure_pursuit_node.py
        recovery_fsm.py
        mode_controller_node.py
    covapsy_bridge/
      package.xml
      setup.py
      covapsy_bridge/
        stm32_bridge_node.py
        dynamixel_steering_node.py
    covapsy_tools/
      package.xml
      setup.py
      covapsy_tools/
        telemetry_logger_node.py
        param_sanity_checker.py
```

## Topic Graph

- scan_filter_node
  - sub: /scan (sensor_msgs/msg/LaserScan)
  - pub: /scan_filtered (sensor_msgs/msg/LaserScan)

- reactive_driver_node
  - sub: /scan_filtered (sensor_msgs/msg/LaserScan)
  - pub: /cmd_drive_reactive (covapsy_interfaces/msg/DriveCommand)
  - pub: /reactive_debug (std_msgs/msg/Float32MultiArray)

- direction_detector_node
  - sub: image topic param (default: /image_raw; real-car profiles: /camera/color/image_raw)
  - sub: /scan_filtered (sensor_msgs/msg/LaserScan)
  - pub: /wrong_direction (std_msgs/msg/Bool)
  - pub: /wrong_direction_confidence (std_msgs/msg/Float32)
  - pub: /track_direction (covapsy_interfaces/msg/DirectionState)

- track_learner_node
  - sub: /scan_filtered (sensor_msgs/msg/LaserScan)
  - sub: /odom (nav_msgs/msg/Odometry)
  - sub: /track_direction (covapsy_interfaces/msg/DirectionState)
  - pub: /track_path (nav_msgs/msg/Path)
  - pub: /track_quality (covapsy_interfaces/msg/TrackQuality)
  - pub: /track_learned (std_msgs/msg/Bool)
  - optional persistence: saves/loads learned track JSON via `track_store_path` (defaults in profiles: `/home/amine/Desktop/covapsy/track/track_sim.json`, `/home/amine/Desktop/covapsy/track/track_real.json`)
  - race profiles set `freeze_after_valid: true` to keep the learned map fixed during traffic racing

- pure_pursuit_node
  - sub: /track_path (nav_msgs/msg/Path)
  - sub: /odom (nav_msgs/msg/Odometry)
  - sub: /scan_filtered (sensor_msgs/msg/LaserScan)
  - pub: /cmd_drive_pursuit (covapsy_interfaces/msg/DriveCommand)
  - uses `track_path` waypoint `pose.position.z` as optional speed hint cap

- mode_controller_node
  - sub: /cmd_drive_reactive (covapsy_interfaces/msg/DriveCommand)
  - sub: /cmd_drive_pursuit (covapsy_interfaces/msg/DriveCommand)
  - sub: /wrong_direction (std_msgs/msg/Bool)
  - sub: /wrong_direction_confidence (std_msgs/msg/Float32)
  - sub: /track_learned (std_msgs/msg/Bool)
  - sub: /track_quality (covapsy_interfaces/msg/TrackQuality)
  - sub: /scan_filtered (sensor_msgs/msg/LaserScan)
  - sub: /wheel_speed (std_msgs/msg/Float32)
  - sub: /rear_obstacle (std_msgs/msg/Bool)
  - sub: /imu/data (sensor_msgs/msg/Imu)
  - pub: /cmd_drive (covapsy_interfaces/msg/DriveCommand)
  - pub: /car_mode (std_msgs/msg/String)
  - pub: /recovery_state (covapsy_interfaces/msg/RecoveryState)
  - pub: /race_telemetry (covapsy_interfaces/msg/RaceTelemetry)
  - race behavior: pursuit tracking with reactive LiDAR obstacle-avoidance blend and safety caps

- stm32_bridge_node
  - sub: /cmd_drive (covapsy_interfaces/msg/DriveCommand)
  - sub: /race_start (std_msgs/msg/Bool)
  - sub: /race_stop (std_msgs/msg/Bool)
  - sub: /car_mode (std_msgs/msg/String)
  - sub: /race_telemetry (covapsy_interfaces/msg/RaceTelemetry)
  - sub: /recovery_state (covapsy_interfaces/msg/RecoveryState)
  - sub: /steering_status (std_msgs/msg/String)
  - pub: /wheel_speed (std_msgs/msg/Float32)
  - pub: /rear_obstacle (std_msgs/msg/Bool)
  - pub: /bridge_status (std_msgs/msg/String)
  - pub: /stm32/cmd_drive (covapsy_interfaces/msg/DriveCommand)
  - runtime: USB serial transport to STM32 (`/dev/stm32_mcu`, 115200, CSV `CMD`/`LCD` out + `TEL` in)
  - LCD bridge: builds a 4x16 operations status page and sends `LCD,<seq>,l1|l2|l3|l4` at low rate
  - supports `external_steering_mode=true` for real-car split actuation

- dynamixel_steering_node
  - sub: /cmd_drive (covapsy_interfaces/msg/DriveCommand)
  - sub: /race_start (std_msgs/msg/Bool)
  - sub: /race_stop (std_msgs/msg/Bool)
  - pub: /steering_status (std_msgs/msg/String)
  - pub: /steering_present_position (std_msgs/msg/Int32)
  - runtime: DYNAMIXEL Protocol 2.0 to XL430 via Waveshare USB adapter (`/dev/waveshare_servo`)

## Parameter File Plan

- learn_sim.yaml
  - low-speed LEARN defaults for Webots.
- race_sim.yaml
  - RACE defaults for Webots with pursuit nominal + safety overlay.
- learn_real.yaml
  - conservative LEARN defaults for hardware (includes STM32 OLED relay params).
- race_real.yaml
  - conservative RACE defaults for hardware (includes STM32 OLED relay params).

Behavior is centralized in these files, not launch arguments.

## Implementation Order (Applied)

1. New package/file tree
2. Topics and interface messages
3. Node skeletons
4. Reactive driving
5. Recovery FSM
6. Direction detection
7. Track learner
8. Pure pursuit
9. Mode controller
10. STM32 bridge
11. Launch files and config
12. README and runbook
