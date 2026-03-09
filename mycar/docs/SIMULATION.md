# Simulation Guide

## Mode A: Standalone Webots (No ROS, Windows-Friendly)
This mode runs everything inside Webots with a pure Python controller.

### Prerequisites
- Webots R2025a.
- No ROS installation required.

### Start (PowerShell, from `mycar`)
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1
```

Enable standalone runtime logs from launch:
```powershell
$env:COVAPSY_STANDALONE_LOGS=1
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1
```

Force LiDAR-only driving (disable RGB/depth fusion):
```powershell
$env:COVAPSY_USE_CAMERA_GUIDANCE=0
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1
```

Keep RGB guidance but disable depth influence:
```powershell
$env:COVAPSY_USE_CAMERA_GUIDANCE=1
$env:COVAPSY_USE_DEPTH_GUIDANCE=0
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1
```

If Webots is not in the default location:
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1 -WebotsExe "C:\Path\To\webotsw.exe"
```
Common Webots Windows install path:
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_webots_standalone.ps1 -WebotsExe "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"
```

### World/controller used
- World: `simulation/webots/worlds/Piste_CoVAPSy_2025a_standalone.wbt`
- Controller: `simulation/webots/controllers/covapsy_standalone/covapsy_standalone.py`
- Robot name: `TT02_standalone`
- Sensors used by standalone controller:
  - LiDAR (`RpLidarA2`) for reactive gap following.
  - Simulated RealSense RGB-D (`RealSenseRGB`, `RealSenseDepth`) for red/green wall guidance and direction sanity.

### Keyboard controls
- `A`: start autonomous mode
- `N`: stop autonomous mode
- `S`: toggle advanced gap follower / simple baseline
- `R`: short reverse maneuver
- `+` / `-`: increase or decrease speed cap
- `L`: toggle runtime logs on/off

## Mode B: ROS2-In-The-Loop (Ubuntu)
Use this mode when validating ROS2 topics and launch files.

### Prerequisites
- Ubuntu 24.04 with ROS2 Jazzy.
- Webots R2025a.
- `mycar/ros2_ws` built at least once.

### 1. Build workspace
```bash
cd mycar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Start ROS2 autonomy stack
```bash
ros2 launch covapsy_bringup sim_webots.launch.py
```

Enable runtime logs directly from launch:
```bash
ros2 launch covapsy_bringup sim_webots.launch.py enable_runtime_logs:=true runtime_log_period_s:=1.0
```

Default behavior:
- `initial_mode=REACTIVE`
- pure pursuit disabled (`enable_pure_pursuit:=false`)
- no hardware bridge node

### 3. Start Webots world
Use the same sourced terminal so the controller can import ROS2 Python packages:
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_ros2.wbt
```

### 4. Verify ROS2 topic flow
```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /cmd_vel_reactive
ros2 topic hz /cmd_vel
ros2 topic echo /car_mode --once
ros2 topic echo /mcu_status --once
```

Expected:
- `/scan` and `/scan_filtered` publish continuously.
- `/cmd_vel_reactive` is non-zero while driving.
- `/cmd_vel` follows mode controller output.
- `/mcu_status` contains `backend=webots_ros2;ok=1`.

## Mode C: ROS2 Multi-Car Benchmark (Ubuntu)
Use this mode to stress-test tactical behavior against other cars.

### 1. Launch ROS2 stack (multi-car tactical profile)
```bash
ros2 launch covapsy_bringup sim_webots_multicar.launch.py race_profile:=RACE_STABLE traffic_mode:=balanced
```

### 2. Start multi-car world
```bash
webots mycar/simulation/webots/worlds/Piste_CoVAPSy_2025a_multicar_ros2.wbt
```

### 3. Tactical checks
```bash
ros2 topic hz /cmd_vel_tactical
ros2 topic echo /opponent_confidence --once
ros2 topic echo /traffic_state --once
```

Toggle runtime logs during run:
```bash
ros2 param set /runtime_monitor enable_logs true
ros2 param set /runtime_monitor enable_logs false
```

Expected:
- Ego car (`TT02_ego_ros2`) is ROS2-controlled.
- Opponent cars run scripted controllers in the same world.
- `/cmd_vel_tactical` is produced when traffic/opponents are detected.
