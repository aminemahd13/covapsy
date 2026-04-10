# Pi / ROS Integration (USB Backend)

This is the real-car integration flow after firmware flashing.

## 1. Pi Base Setup

On Raspberry Pi:

```bash
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

These scripts install serial dependencies and create a stable STM32 symlink (`/dev/stm32_mcu`).

## 2. Verify USB Device Binding

Plug STM32 into USB and verify:

```bash
ls -l /dev/stm32_mcu
```

If missing, inspect USB identity and update udev matching as needed:

```bash
udevadm info -a -n /dev/ttyACM0 | less
```

## 3. Deploy and Build ROS Workspace

From development machine:

```bash
bash scripts/deploy_to_pi.sh <pi_ip_or_hostname>
```

On Pi:

```bash
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

## 4. Safe Bringup (USB)

```bash
ros2 launch covapsy_bringup car_safe.launch.py
```

Expected status transition on `/bridge_status`:

- starts as `USB_DISCONNECTED` or `USB_TIMEOUT` until telemetry arrives
- moves to `WAIT_START` or `RUN` when link is healthy

## 5. Runtime Checks

```bash
ros2 topic echo /bridge_status --once
ros2 topic echo /wheel_speed --once
ros2 topic echo /rear_obstacle --once
```

Arm race control and publish low-speed command path:

```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
```

Stop command source and confirm watchdog neutral behavior within timeout,
then stop race control:

```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```
