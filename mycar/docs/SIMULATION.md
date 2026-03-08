# Simulation Guide (Webots + ROS2)

## Goal
Run the same ROS2 autonomy nodes used on the real car, but with Webots sensors and actuators.

## Prerequisites
- Ubuntu 24.04 with ROS2 Jazzy installed.
- Webots R2025a installed.
- `mycar/ros2_ws` built once.

## 1. Build Workspace
```bash
cd mycar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Start ROS2 Autonomy Stack
```bash
ros2 launch covapsy_bringup sim_webots.launch.py
```

Default behavior:
- `initial_mode=REACTIVE`
- pure pursuit disabled (`enable_pure_pursuit:=false`)
- no hardware bridge node

## 3. Start Webots World
Use the same sourced terminal so Webots controller can import ROS2 Python packages:
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

World details:
- One car: `TT02_ros2`
- Controller: `covapsy_ros2_bridge`
- Sensors used by bridge:
  - LiDAR `RpLidarA2` -> `/scan`
  - GPS + IMU -> `/odom`

## 4. Verify Topic Flow
```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel
ros2 topic echo /car_mode --once
ros2 topic echo /mcu_status --once
```

Expected:
- `/scan` and `/scan_filtered` publishing continuously.
- `/cmd_vel_reactive` non-zero while driving.
- `/cmd_vel` following mode controller output.
- `/mcu_status` contains `backend=webots_ros2;ok=1`.

## 5. Optional Parameter Overrides
```bash
ros2 launch covapsy_bringup sim_webots.launch.py max_speed:=1.0 initial_mode:=REACTIVE
```

## 6. Standalone Mode (No ROS2)
For pure Webots local testing:
- World: `simulation/webots/worlds/Piste_CoVAPSy_2025a.wbt`
- Controller: `simulation/webots/controllers/covapsy_controller/covapsy_controller.py`

Keyboard:
- `A`: toggle autonomous mode.
- `N`: stop.
- `R`: reverse.
- `S`: toggle simple/advanced control logic.
