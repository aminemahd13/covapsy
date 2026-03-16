# Pi / ROS Integration (SPI Backend)

This is the real-car integration flow after firmware flashing.

## 1. Pi Base Setup

On Raspberry Pi:

```bash
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

These scripts enable `spi` and install `spidev` dependencies.

## 2. HAT Jumper Position

For SPI backend with STM32-generated PWM:

- propulsion PWM source jumper -> STM32 side (left)
- steering PWM source jumper -> STM32 side (left)

Reference:

- `docs/HAT_JUMPERS.md`

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

## 4. Safe Bringup (SPI)

```bash
ros2 launch covapsy_bringup car_safe.launch.py backend:=spi
```

Expected status:

- `/mcu_status` includes `backend=spi;ok:spi:...`
- no `error:spi_open` or `error:spi_xfer`

## 5. Runtime Checks

```bash
ros2 topic echo /mcu_status --once
ros2 topic echo /wheel_speed --once
ros2 topic echo /rear_obstacle --once
```

Arm race control and then apply low-speed reactive command:

```bash
ros2 topic pub /race_start std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /cmd_vel_reactive geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 5
```

Stop publisher and confirm watchdog stop within `0.25` s.
Then stop race control:

```bash
ros2 topic pub /race_stop std_msgs/msg/Bool "{data: true}" --once
```
