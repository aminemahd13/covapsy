# Pi / ROS Integration (USB Backend)

This is the real-car integration flow after firmware flashing.

## 1. Pi Base Setup

On Raspberry Pi:

```bash
bash scripts/setup_pi.sh
bash scripts/install_ros2.sh
```

These scripts install serial dependencies and create a stable STM32 symlink (`/dev/stm32_mcu`).

Optional deterministic STM32 binding (recommended when multiple ST devices are connected):

```bash
export STM32_SERIAL="<udevadm-serial-for-your-board>"
bash scripts/setup_pi.sh
```

## 2. Verify Direct Device First (`/dev/ttyACM*`)

Plug STM32 and identify the direct ACM device:

```bash
ls -l /dev/ttyACM*
STM_TTY=/dev/ttyACM0   # adjust to detected device
```

Confirm idle telemetry appears before ROS:

```bash
stty -F "$STM_TTY" 115200 -echo -icanon -icrnl -ixon
timeout 2 cat "$STM_TTY"
```

Expected: recurring `TEL,...` lines.

In another terminal, send a valid command burst:

```bash
for i in $(seq 1 20); do
  printf "CMD,%d,0.000,0.100,1,0\n" "$i" > "$STM_TTY"
  sleep 0.02
done
```

Expected: telemetry continues without parser errors.

## 3. Verify Symlink Binding (`/dev/stm32_mcu`)

Plug STM32 into USB and verify:

```bash
ls -l /dev/stm32_mcu
```

If missing or ambiguous, inspect identity and update udev matching:

```bash
udevadm info -a -n /dev/ttyACM0 | less
```

## 4. Deploy and Build ROS Workspace

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

## 5. ROS USB Bringup

Serial-first ROS check (bridge only, direct device once):

```bash
ros2 run covapsy_bridge stm32_bridge_node --ros-args -p serial_device:="$STM_TTY" -p serial_baud:=115200
```

Then run normal profile using `/dev/stm32_mcu`:

```bash
ros2 launch covapsy_bringup car_safe.launch.py
```

Expected status transition on `/bridge_status`:

- starts as `USB_DISCONNECTED` or `USB_TIMEOUT` until telemetry arrives
- moves to `WAIT_START` or `RUN` when link is healthy

## 6. Runtime Checks

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
