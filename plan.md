# COVAPSY Autonomous Racing Car — Complete Implementation Plan

**Platform:** Raspberry Pi 5 (2GB) · STM32 on custom Pi HAT (with 1.6" TFT) · RPLIDAR A2M12 · Intel RealSense RGB-D · Rear IR presence sensors PCB
**Target:** 6th edition COVAPSY, ENS Paris-Saclay, April 11, 2026
**Purpose:** This document is a step-by-step specification for an AI coding agent to implement the full software stack from bare OS to race-ready autonomy.

---

## TABLE OF CONTENTS

1. [Hardware Inventory and Wiring Map](#1-hardware-inventory-and-wiring-map)
2. [OS Installation and Boot Configuration](#2-os-installation-and-boot-configuration)
3. [ROS2 Jazzy Stack Installation](#3-ros2-jazzy-stack-installation)
4. [STM32 HAT Firmware and Communication](#4-stm32-hat-firmware-and-communication)
5. [RPLIDAR A2M12 Driver Setup](#5-rplidar-a2m12-driver-setup)
6. [Intel RealSense RGB-D Integration](#6-intel-realsense-rgb-d-integration)
7. [Rear IR Presence Sensor Integration](#7-rear-ir-presence-sensor-integration)
8. [TFT Display Integration](#8-tft-display-integration)
9. [Simulation Environment Setup](#9-simulation-environment-setup)
10. [Perception Pipeline: LiDAR Processing](#10-perception-pipeline-lidar-processing)
11. [Perception Pipeline: Camera Processing](#11-perception-pipeline-camera-processing)
12. [SLAM and Localization](#12-slam-and-localization)
13. [Reactive Navigation: Follow-the-Gap](#13-reactive-navigation-follow-the-gap)
14. [Map-Based Navigation: Trajectory Tracking](#14-map-based-navigation-trajectory-tracking)
15. [Dual-Mode State Machine](#15-dual-mode-state-machine)
16. [Trackside Computer Setup](#16-trackside-computer-setup)
17. [ROS2 Workspace Structure](#17-ros2-workspace-structure)
18. [Launch Files](#18-launch-files)
19. [Parameter Tuning Guide](#19-parameter-tuning-guide)
20. [Testing and Validation Protocol](#20-testing-and-validation-protocol)
21. [Race-Day Checklist](#21-race-day-checklist)

---

## 1. Hardware Inventory and Wiring Map

### 1.1 Complete parts list

| Component | Interface | Pi 5 connection | Notes |
|-----------|-----------|-----------------|-------|
| Raspberry Pi 5 (2GB) | — | — | Main compute board |
| STM32 HAT PCB | GPIO header (SPI/UART/I2C) | Mounted directly on Pi 5 40-pin header | Controls ESC + steering servo, reads encoder, has 1.6" TFT |
| RPLIDAR A2M12 | USB (CP2102 serial) | USB 2.0 port (leave USB 3.0 for RealSense) | 360° 2D LiDAR, 256kbps, 10 Hz scan rate |
| Intel RealSense D4xx | USB 3.0 (mandatory) | USB 3.0 port (blue) | Depth + RGB. USB 2.0 is NOT supported for depth streaming |
| Rear IR sensor PCB | Connected to STM32 HAT or separate UART | Via STM32 HAT (the STM32 reads IR and forwards to Pi) | Detects cars approaching from behind |
| Active Cooler | Dedicated 4-pin fan header | Pi 5 fan connector (not GPIO) | Non-negotiable for sustained load |
| 5V/5A Buck Converter | USB-C output | Pi 5 USB-C power input | e.g., Pololu D36V50F5, input from 7.2V NiMH |
| NiMH Battery 7.2V | — | Via buck converter | 5000 mAh max per competition rules |
| MicroSD or NVMe | SD slot or M.2 HAT+ | — | NVMe strongly recommended for swap performance |

### 1.2 Wiring schematic (logical)

```
NiMH 7.2V Battery
    ├──> ESC (motor power, controlled by STM32 PWM)
    └──> 5V/5A Buck Converter ──> USB-C ──> Raspberry Pi 5
                                                ├── 40-pin GPIO ──> STM32 HAT PCB
                                                │                    ├── ESC PWM out
                                                │                    ├── Servo PWM out
                                                │                    ├── Encoder input
                                                │                    ├── 1.6" TFT (SPI)
                                                │                    └── Rear IR sensor PCB (digital input or I2C)
                                                ├── USB 3.0 (blue) ──> Intel RealSense
                                                ├── USB 2.0 ──> RPLIDAR A2M12
                                                └── WiFi ──> Trackside laptop (ROS2 DDS)
```

### 1.3 Critical power notes

- Pi 5 MUST be powered via USB-C, not GPIO 5V pins (GPIO power bypasses PMIC, prevents auto-boot).
- Set EEPROM: `PSU_MAX_CURRENT=5000` so Pi knows the supply can deliver 5A.
- Set `config.txt`: `usb_max_current_enable=1` to allow full USB current draw (RPLidar motor needs ~200-400mA).
- RealSense draws ~700mA on USB 3.0; combined with LiDAR this is within the 1.6A USB budget at 5A supply.

---

## 2. OS Installation and Boot Configuration

### 2.1 Flash Ubuntu Server 24.04 LTS

```bash
# On your laptop:
# 1. Download Ubuntu Server 24.04 for Raspberry Pi from https://ubuntu.com/download/raspberry-pi
# 2. Flash to microSD (or NVMe if using an adapter) with Raspberry Pi Imager
# 3. In Imager advanced settings:
#    - Set hostname: covapsy-car
#    - Enable SSH with password authentication
#    - Set username: covapsy, password: <your-password>
#    - Configure WiFi: SSID and password for your lab/competition network
#    - Set locale and timezone: Europe/Paris
```

### 2.2 First boot and system configuration

SSH into the Pi after first boot:

```bash
ssh covapsy@covapsy-car.local

# Update system
sudo apt update && sudo apt full-upgrade -y

# Disable unnecessary services to save RAM
sudo systemctl disable snapd snapd.socket snapd.seeded
sudo systemctl disable unattended-upgrades
sudo systemctl disable ModemManager
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# Install essential tools
sudo apt install -y htop tmux git build-essential cmake python3-pip \
  python3-venv i2c-tools can-utils minicom screen
```

### 2.3 Boot configuration: `/boot/firmware/config.txt`

Add/modify these lines at the end of the file:

```ini
# === COVAPSY Pi 5 Configuration ===

# Enable UART0 on GPIO 14(TX)/15(RX) for STM32 communication
dtparam=uart0=on

# Raise USB current budget (needed for RPLidar + RealSense)
usb_max_current_enable=1

# Enable PCIe Gen 3 for NVMe speed (optional, only if M.2 HAT installed)
dtparam=pciex1_gen=3

# Enable SPI for TFT display on HAT (if TFT uses SPI)
dtparam=spi=on

# Enable I2C (if rear IR sensor PCB uses I2C)
dtparam=i2c_arm=on

# Disable Bluetooth to free UART (if BT not needed)
# dtoverlay=disable-bt

# GPU memory - Pi 5 uses dynamic allocation, no gpu_mem setting needed
```

### 2.4 EEPROM configuration

```bash
sudo -E rpi-eeprom-config --edit
# Add or modify:
# PSU_MAX_CURRENT=5000
# BOOT_ORDER=0xf416  (NVMe first if present, then SD)
```

### 2.5 Swap configuration

For 2GB RAM, swap is essential during compilation and a safety net during runtime:

```bash
# If using NVMe:
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Also enable ZRAM for compressed in-memory swap
sudo apt install -y zram-tools
echo -e "ALGO=lz4\nPERCENT=50" | sudo tee /etc/default/zramswap
sudo systemctl enable zramswap
sudo systemctl restart zramswap
```

### 2.6 Thermal verification

```bash
# Run a stress test and monitor temperature
sudo apt install -y stress-ng
# In one terminal:
watch -n 1 'vcgencmd measure_temp && vcgencmd get_throttled'
# In another terminal:
stress-ng --cpu 4 --timeout 300s
# PASS criteria: temperature stays below 80°C for 5 minutes
# If it exceeds 80°C, the active cooler is not properly mounted
```

---

## 3. ROS2 Jazzy Stack Installation

### 3.1 Install ROS2 Jazzy from apt

```bash
# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Install ros-base (NO desktop packages on the car)
sudo apt install -y ros-jazzy-ros-base

# Install competition-specific packages
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rplidar-ros \
  ros-jazzy-micro-ros-agent \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf-transformations \
  ros-jazzy-robot-localization \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-ackermann-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 3.2 Install Intel RealSense ROS2 wrapper

The apt binary may not be available for Jazzy. Build from source:

```bash
# Install librealsense2 SDK
sudo apt install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

cd ~/
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
mkdir build && cd build
# Build with -j2 to avoid OOM on 2GB Pi
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_WITH_OPENMP=ON \
  -DFORCE_RSUSB_BACKEND=ON
make -j2
sudo make install

# Then build the ROS2 wrapper in the workspace (see section 17)
```

### 3.3 Shell environment

Add to `~/.bashrc`:

```bash
# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# DDS implementation — CycloneDDS is lighter than FastDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Reduce DDS discovery traffic (important on constrained hardware)
export ROS_LOCALHOST_ONLY=0  # Set to 1 if NOT using trackside computer
export ROS_DOMAIN_ID=42      # Unique ID for your team

# Workspace (created in section 17)
if [ -f ~/covapsy_ws/install/setup.bash ]; then
  source ~/covapsy_ws/install/setup.bash
fi
```

### 3.4 CycloneDDS tuning for low-memory operation

Create `~/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
    </General>
    <Internal>
      <MinimumSocketReceiveBufferSize>64kB</MinimumSocketReceiveBufferSize>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
```

Add to `~/.bashrc`:

```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

---

## 4. STM32 HAT Firmware and Communication

### 4.1 Architecture overview

The professor-provided STM32 HAT PCB sits directly on the Pi 5 GPIO header. It contains:
- STM32 microcontroller (likely NUCLEO-G431KB or similar Cortex-M4)
- Motor driver / ESC PWM output
- Steering servo PWM output
- Encoder input (optical fork sensor)
- 1.6" TFT screen (SPI-connected to STM32, not to Pi directly)
- Connection to rear IR sensor PCB

Communication between Pi 5 and STM32 uses **UART** via GPIO 14 (TX) and GPIO 15 (RX) at `/dev/ttyAMA0`.

### 4.2 Communication protocol options

**Option A: micro-ROS (recommended if STM32 firmware supports it)**

On the STM32 side, use `micro_ros_stm32cubemx_utils` with FreeRTOS:
- Subscribe to: `/cmd_drive` (AckermannDriveStamped or custom message with steering_angle + speed)
- Publish: `/wheel_odom` (twist with linear.x = speed from encoder), `/rear_ir` (Bool or Range), `/mcu_status` (diagnostic)

On the Pi 5 side:

```bash
# Launch micro-ROS agent for UART
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyAMA0 -b 115200
```

**Option B: Custom binary protocol (if STM32 firmware is fixed/proprietary)**

If the professor's firmware uses a custom serial protocol, write a ROS2 bridge node:

```python
# File: covapsy_ws/src/covapsy_bridge/covapsy_bridge/stm32_bridge_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import serial
import struct

class STM32BridgeNode(Node):
    """Bridge between ROS2 topics and STM32 HAT custom serial protocol."""

    def __init__(self):
        super().__init__('stm32_bridge')

        # Serial port to STM32 via UART on GPIO 14/15
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

        # Subscribers: commands FROM ROS2 TO STM32
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishers: data FROM STM32 TO ROS2
        self.speed_pub = self.create_publisher(Float32, '/wheel_speed', 10)
        self.rear_ir_pub = self.create_publisher(Bool, '/rear_obstacle', 10)

        # Timer to read STM32 data at 100 Hz
        self.create_timer(0.01, self.read_stm32)

        # Watchdog: if no command in 250ms, send zero
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.05, self.watchdog_check)

    def cmd_callback(self, msg: Twist):
        """Send steering and speed to STM32."""
        self.last_cmd_time = self.get_clock().now()
        steering = msg.angular.z   # radians, mapped to servo PWM by STM32
        speed = msg.linear.x       # m/s, mapped to ESC PWM by STM32

        # Pack as: [0xAA, 0x55, CMD_DRIVE, steering_float, speed_float, CRC8]
        payload = struct.pack('<ff', steering, speed)
        frame = bytes([0xAA, 0x55, 0x01]) + payload
        frame += bytes([self._crc8(frame)])
        self.ser.write(frame)

    def read_stm32(self):
        """Read telemetry frames from STM32."""
        while self.ser.in_waiting >= 8:
            if self.ser.read(1) == b'\xAA':
                if self.ser.read(1) == b'\x55':
                    msg_type = self.ser.read(1)[0]
                    if msg_type == 0x02:  # Speed telemetry
                        data = self.ser.read(5)  # 4 bytes float + CRC
                        speed = struct.unpack('<f', data[:4])[0]
                        msg = Float32()
                        msg.data = speed
                        self.speed_pub.publish(msg)
                    elif msg_type == 0x03:  # Rear IR
                        data = self.ser.read(2)  # 1 byte bool + CRC
                        msg = Bool()
                        msg.data = bool(data[0])
                        self.rear_ir_pub.publish(msg)

    def watchdog_check(self):
        """Send zero command if no cmd received recently."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.25:
            payload = struct.pack('<ff', 0.0, 0.0)
            frame = bytes([0xAA, 0x55, 0x01]) + payload
            frame += bytes([self._crc8(frame)])
            self.ser.write(frame)

    @staticmethod
    def _crc8(data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**IMPORTANT:** You MUST determine the actual protocol of the professor's STM32 firmware. Inspect any provided documentation, `.c`/`.h` files, or use `minicom -b 115200 -D /dev/ttyAMA0` to observe the raw bytes the STM32 sends at boot. Adapt the bridge node to match the actual protocol exactly.

### 4.3 STM32 firmware responsibilities (what the STM32 must do)

The STM32 on the HAT is the real-time safety layer. Its firmware MUST:

1. **Generate 50 Hz PWM** for steering servo (pulse 1.0–2.0 ms) and ESC (pulse 1.0–2.0 ms).
2. **Read encoder** via hardware timer encoder mode at ≥100 Hz to compute wheel speed.
3. **Run PID speed control** at 100–200 Hz loop rate, tracking the speed setpoint from Pi.
4. **Implement watchdog timeout**: if no valid command from Pi within 250 ms, set speed=0 and steering=center.
5. **Read rear IR sensors** and forward state to Pi over the serial link.
6. **Drive 1.6" TFT display** showing status info (IP address, mode, speed, battery voltage).

---

## 5. RPLIDAR A2M12 Driver Setup

### 5.1 Hardware connection

- Connect RPLIDAR A2M12 to a Pi 5 **USB 2.0 port** (reserve USB 3.0 for RealSense).
- The RPLidar appears as `/dev/ttyUSB0` (CP2102 USB-serial adapter).
- Create a udev rule for stable naming:

```bash
# Find the serial number
udevadm info -a /dev/ttyUSB0 | grep serial

# Create udev rule
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="rplidar", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
# Verify: ls -la /dev/rplidar
```

### 5.2 ROS2 launch configuration

```python
# File: covapsy_ws/src/covapsy_bringup/launch/lidar.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',  # Best for indoor racing
                'scan_frequency': 10.0,
            }],
            output='screen'
        ),
    ])
```

### 5.3 Verification

```bash
# Launch the driver
ros2 launch covapsy_bringup lidar.launch.py

# In another terminal, verify data:
ros2 topic echo /scan --once
# Should show sensor_msgs/msg/LaserScan with ~360+ ranges

ros2 topic hz /scan
# Should show ~10 Hz
```

---

## 6. Intel RealSense RGB-D Integration

### 6.1 Critical hardware requirement

The Intel RealSense D4xx series **requires USB 3.0**. The Pi 5 has USB 3.0 ports (the blue ones). This was impossible on Pi 3 (USB 2.0 only). Connect the RealSense to a USB 3.0 port.

Create a udev rule:

```bash
# Install RealSense udev rules
cd ~/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 6.2 Verify standalone operation

```bash
# Test with rs-enumerate-devices (installed with librealsense)
rs-enumerate-devices

# Test basic streaming
rs-depth  # Should display depth values to terminal
```

### 6.3 Build RealSense ROS2 wrapper (in workspace, see section 17)

```bash
# In the covapsy_ws workspace
cd ~/covapsy_ws/src
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-development

cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2  # Limit parallel jobs for 2GB RAM
```

### 6.4 Launch configuration for competition use

Run at **reduced resolution** to conserve RAM and CPU. Depth at 424×240 at 15 FPS is sufficient for obstacle detection:

```python
# File: covapsy_ws/src/covapsy_bringup/launch/realsense.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '424x240x15',  # Low res to save bandwidth
                'depth_module.depth_profile': '424x240x15',
                'align_depth.enable': True,
                'pointcloud.enable': False,  # Too heavy for Pi 5 2GB
                'decimation_filter.enable': True,
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
            }],
            output='screen'
        ),
    ])
```

### 6.5 Memory warning

The RealSense driver + depth processing adds ~200–400 MB of RAM usage. On 2GB, running RealSense simultaneously with SLAM + Nav2 will push into swap. Strategy:
- Run RealSense only when needed (e.g., for border color detection or close-range obstacle verification).
- OR offload RealSense processing to the trackside laptop by streaming compressed images over WiFi.
- OR if RAM allows at runtime, run at minimum resolution (424×240×6fps for depth).

---

## 7. Rear IR Presence Sensor Integration

### 7.1 Architecture

The rear IR sensor PCB connects to the STM32 HAT (not directly to Pi GPIO). The STM32 reads the digital/analog IR outputs and forwards the obstacle-behind state to the Pi over the UART serial link.

### 7.2 ROS2 topic specification

The STM32 bridge node (section 4.2) publishes:

- **Topic:** `/rear_obstacle`
- **Type:** `std_msgs/msg/Bool`
- **Semantics:** `True` = obstacle detected behind the car, `False` = clear

### 7.3 Usage in navigation

The rear obstacle signal is used in two scenarios:
1. **Reverse maneuver safety**: Before reversing when blocked, check `/rear_obstacle`. If True, do not reverse; wait or attempt forward steering instead.
2. **Race awareness**: If a faster car is behind (IR triggered), the planning node can choose to hold its racing line rather than making sudden moves.

---

## 8. TFT Display Integration

### 8.1 Architecture

The 1.6" TFT screen on the HAT is driven by the STM32, not by the Pi. The Pi sends display update commands over the same UART link.

### 8.2 Display information during operation

The STM32 should show on the TFT:
- **Boot:** Pi IP address (sent from Pi to STM32 at startup)
- **Running:** Current mode (REACTIVE / MAPPING / RACING), speed (m/s), battery voltage
- **Error:** Fault codes (LIDAR_FAIL, REALSENSE_FAIL, CMD_TIMEOUT)

### 8.3 Pi-side status sender

```python
# File: covapsy_ws/src/covapsy_bridge/covapsy_bridge/tft_status_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import subprocess

class TFTStatusNode(Node):
    def __init__(self):
        super().__init__('tft_status')
        self.mode_sub = self.create_subscription(String, '/car_mode', self.mode_cb, 10)
        self.speed_sub = self.create_subscription(Float32, '/wheel_speed', self.speed_cb, 10)
        self.mode = 'BOOT'
        self.speed = 0.0
        self.create_timer(1.0, self.send_status)  # Update display at 1 Hz

    def mode_cb(self, msg):
        self.mode = msg.data

    def speed_cb(self, msg):
        self.speed = msg.data

    def send_status(self):
        # Send display command to STM32 via the bridge
        # Format: TFT_UPDATE,<mode>,<speed>,<ip>
        ip = self._get_ip()
        status_msg = String()
        status_msg.data = f"TFT:{self.mode},{self.speed:.1f},{ip}"
        # This gets picked up by the STM32 bridge and forwarded over UART
        self.status_pub = self.create_publisher(String, '/tft_command', 10)
        self.status_pub.publish(status_msg)

    def _get_ip(self):
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=1)
            return result.stdout.strip().split()[0]
        except Exception:
            return '0.0.0.0'
```

---

## 9. Simulation Environment Setup

Simulation runs on your **development laptop** (not on the Pi). Two simulators are used:

### 9.1 Simulator A: Official COVAPSY Webots Simulator (physics-accurate)

This is the official simulator provided by ENS Paris-Saclay with a TT-02 model and LiDAR.

```bash
# On your LAPTOP (Linux, Windows, or macOS):

# 1. Install Webots R2023b or later
#    Download from: https://cyberbotics.com/
#    Linux: sudo snap install webots  (or use the tarball)

# 2. Download the official COVAPSY simulator project
git clone https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay.git
cd CourseVoituresAutonomesSaclay/Simulateur/

# 3. Unzip the base project
#    Look for: Simulateur_CoVAPSy_Webots2023b_Base.zip
unzip Simulateur_CoVAPSy_Webots2023b_Base.zip

# 4. Open in Webots
#    File > Open World > select the .wbt file from the unzipped project
```

The Webots simulator includes:
- TT-02 car model with Ackermann steering physics
- Simulated LiDAR (configurable to match A2M12 specs)
- Track environment with green/red borders
- Python or C controller interface

**Controller development workflow:**
1. Write your control algorithm as a Webots Python controller.
2. The controller reads simulated LiDAR scan data (same format as real RPLidar).
3. The controller outputs steering angle and speed commands.
4. Once validated in simulation, port the logic to a ROS2 node.

### 9.2 Simulator B: F1TENTH Gym (fast iteration, ROS2-native)

For rapid algorithm development with ROS2 integration:

```bash
# On your LAPTOP:
# Option 1: Native install (Ubuntu with ROS2)
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

cd ~/sim_ws
source /opt/ros/jazzy/setup.bash  # or humble
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install

# Edit the map path in config/sim.yaml:
# map_path: '<full_path>/sim_ws/src/f1tenth_gym_ros/maps/levine'

source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Option 2: Docker (recommended if ROS2 not installed on laptop)
cd ~/sim_ws/src/f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
# Inside container:
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

F1TENTH Gym publishes:
- `/scan` (sensor_msgs/LaserScan) — simulated LiDAR
- `/ego_racecar/odom` (nav_msgs/Odometry) — ground truth odometry
- Subscribes to `/drive` (AckermannDriveStamped)

### 9.3 Simulation-to-real topic mapping

To make the same ROS2 nodes work in both simulation and real car, use topic remapping:

| Function | Simulation topic | Real car topic | Remapping strategy |
|----------|-----------------|----------------|-------------------|
| LiDAR scan | `/scan` | `/scan` | Same (no remap needed) |
| Drive command | `/drive` | `/cmd_vel` or `/cmd_drive` | Remap in launch file |
| Odometry | `/ego_racecar/odom` | `/odom` | Remap in launch file |
| Rear obstacle | N/A (not simulated) | `/rear_obstacle` | Add dummy publisher in sim |

### 9.4 Creating custom COVAPSY-like tracks for F1TENTH Gym

The F1TENTH Gym loads tracks from PNG/PGM image files. Create custom tracks:

```bash
# Use any image editor (GIMP, Inkscape) to draw a track:
# - White pixels = drivable area
# - Black pixels = walls
# - Save as .pgm (PGM format)

# Create accompanying .yaml metadata file:
cat > my_covapsy_track.yaml << 'EOF'
image: my_covapsy_track.pgm
resolution: 0.05  # meters per pixel
origin: [-5.0, -5.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
EOF

# Update sim.yaml to use your track:
# map_path: '/path/to/my_covapsy_track'
```

Create **at least 5 different track layouts** varying:
- Turn radii (test minimum 400mm at 1/10 scale = 40mm in sim)
- Chicanes and S-curves
- Track width (test minimum 800mm at 1/10 scale)
- Long straights followed by sharp turns

### 9.5 Sorbonne University reference: complete Webots + ROS simulation

Study and adapt from the Sorbonne team's open-source implementation:

```bash
git clone https://github.com/SU-Bolides/Course_2025.git
# Contains:
# - simulation_env/  → Webots simulation with TT-02
# - course_2025_slam_pkgs/  → ROS packages (SLAM navigation)
# - CoVAPSy_STM32/  → STM32 firmware reference
```

---

## 10. Perception Pipeline: LiDAR Processing

### 10.1 Scan preprocessing node

```python
# File: covapsy_ws/src/covapsy_perception/covapsy_perception/scan_filter_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanFilterNode(Node):
    """Preprocess raw LiDAR scans: clip range, apply median filter, remove invalid readings."""

    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('median_window', 3)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def scan_cb(self, msg: LaserScan):
        max_r = self.get_parameter('max_range').value
        min_r = self.get_parameter('min_range').value
        window = self.get_parameter('median_window').value

        ranges = np.array(msg.ranges, dtype=np.float32)

        # Replace inf/nan with max_range
        ranges = np.where(np.isfinite(ranges), ranges, max_r)
        # Clip to valid range
        ranges = np.clip(ranges, min_r, max_r)
        # Median filter to reduce salt-and-pepper noise
        if window > 1:
            padded = np.pad(ranges, window // 2, mode='edge')
            ranges = np.array([
                np.median(padded[i:i + window])
                for i in range(len(ranges))
            ])

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = min_r
        out.range_max = max_r
        out.ranges = ranges.tolist()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanFilterNode())
    rclpy.shutdown()
```

---

## 11. Perception Pipeline: Camera Processing

### 11.1 Border color detection (classical CV, no neural network)

```python
# File: covapsy_ws/src/covapsy_perception/covapsy_perception/border_detect_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class BorderDetectNode(Node):
    """Detect green (right) and red (left) track borders using HSV color filtering."""

    def __init__(self):
        super().__init__('border_detect')
        self.bridge = CvBridge()

        # Subscribe to RealSense color image
        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_cb, 5)
        self.pub = self.create_publisher(Float32, '/camera_steering_offset', 10)

        # HSV ranges for COVAPSY border colors
        # GREEN (RAL 6037): tuned for indoor fluorescent lighting
        self.green_lower = np.array([35, 50, 50])
        self.green_upper = np.array([85, 255, 255])
        # RED (RAL 3020): two ranges because red wraps around H=0/180
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 50, 50])
        self.red_upper2 = np.array([180, 255, 255])

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # Only look at the lower third of the image (track-level)
        roi = frame[2 * h // 3:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detect green (right border)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        # Detect red (left border)
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Compute centroids
        green_cx = self._centroid_x(green_mask)
        red_cx = self._centroid_x(red_mask)

        # Compute steering offset: positive = steer right, negative = steer left
        offset = 0.0
        roi_w = roi.shape[1]
        center = roi_w / 2.0

        if green_cx is not None and red_cx is not None:
            # Both borders visible: steer toward center of the lane
            lane_center = (green_cx + red_cx) / 2.0
            offset = (lane_center - center) / center  # Normalized [-1, 1]
        elif green_cx is not None:
            # Only right border visible: steer left (away from it)
            offset = -0.3
        elif red_cx is not None:
            # Only left border visible: steer right (away from it)
            offset = 0.3

        msg_out = Float32()
        msg_out.data = float(offset)
        self.pub.publish(msg_out)

    @staticmethod
    def _centroid_x(mask):
        moments = cv2.moments(mask)
        if moments['m00'] > 500:  # Minimum area threshold
            return moments['m10'] / moments['m00']
        return None

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BorderDetectNode())
    rclpy.shutdown()
```

---

## 12. SLAM and Localization

### 12.1 SLAM Toolbox configuration

```yaml
# File: covapsy_ws/src/covapsy_bringup/config/slam_toolbox_params.yaml

slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_filtered
    mode: mapping  # Switch to 'localization' after map is built

    # Pi 5 tuned parameters
    max_laser_range: 5.0
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    map_update_interval: 0.3
    resolution: 0.05
    max_queue_size: 10
    stack_size_to_use: 40000000

    # Threading (Pi 5 has 4 cores)
    solver_threads: 2  # Leave 2 cores for Nav2 + drivers

    use_scan_matching: true
    use_scan_barycenter: true
    transform_publish_period: 0.02
    tf_buffer_duration: 30.0
```

### 12.2 SLAM workflow for race day

```bash
# Phase 1: MAPPING (during 3 setup laps)
ros2 launch covapsy_bringup slam.launch.py mode:=mapping
# Drive slowly around the track (0.5 m/s max)
# On trackside laptop, monitor in RViz2

# Phase 2: SAVE MAP
ros2 run nav2_map_server map_saver_cli -f ~/maps/covapsy_track --ros-args -p save_map_timeout:=5000

# Phase 3: Switch to LOCALIZATION mode for racing
ros2 launch covapsy_bringup slam.launch.py mode:=localization map:=~/maps/covapsy_track
```

---

## 13. Reactive Navigation: Follow-the-Gap

### 13.1 Full implementation with Disparity Extension

```python
# File: covapsy_ws/src/covapsy_nav/covapsy_nav/gap_follower_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class GapFollowerNode(Node):
    """Follow-the-Gap with Disparity Extension for reactive obstacle avoidance."""

    def __init__(self):
        super().__init__('gap_follower')

        # Parameters
        self.declare_parameter('car_width', 0.30)        # meters
        self.declare_parameter('max_speed', 2.0)          # m/s
        self.declare_parameter('min_speed', 0.5)          # m/s
        self.declare_parameter('max_range', 5.0)          # meters
        self.declare_parameter('safety_radius', 0.20)     # meters
        self.declare_parameter('disparity_threshold', 0.5) # meters
        self.declare_parameter('steering_gain', 1.0)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.rear_sub = self.create_subscription(
            Bool, '/rear_obstacle', self.rear_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_reactive', 10)

        self.rear_blocked = False

    def rear_cb(self, msg: Bool):
        self.rear_blocked = msg.data

    def scan_cb(self, msg: LaserScan):
        car_width = self.get_parameter('car_width').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        max_range = self.get_parameter('max_range').value
        safety_r = self.get_parameter('safety_radius').value
        disp_thresh = self.get_parameter('disparity_threshold').value
        steer_gain = self.get_parameter('steering_gain').value

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.clip(ranges, 0.0, max_range)
        n = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, n)

        # Step 1: Disparity extension
        half_width = car_width / 2.0
        for i in range(1, n):
            diff = abs(ranges[i] - ranges[i - 1])
            if diff > disp_thresh:
                # Extend the closer reading
                closer_idx = i if ranges[i] < ranges[i - 1] else i - 1
                closer_range = ranges[closer_idx]
                if closer_range > 0.01:
                    extend_angle = np.arctan2(half_width, closer_range)
                    extend_count = int(extend_angle / abs(msg.angle_increment))
                    for j in range(max(0, closer_idx - extend_count),
                                   min(n, closer_idx + extend_count + 1)):
                        ranges[j] = min(ranges[j], closer_range)

        # Step 2: Safety bubble around nearest obstacle
        nearest_idx = np.argmin(ranges)
        nearest_dist = ranges[nearest_idx]
        for i in range(n):
            if nearest_dist > 0.01:
                arc_dist = nearest_dist * abs(angles[i] - angles[nearest_idx])
                if arc_dist < safety_r:
                    ranges[i] = 0.0

        # Step 3: Find largest gap
        nonzero = ranges > 0.01
        gaps = []
        start = None
        for i in range(n):
            if nonzero[i] and start is None:
                start = i
            elif not nonzero[i] and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, n - 1))

        if not gaps:
            # No gap found: emergency stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Step 4: Choose best point in largest gap
        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_slice = slice(best_gap[0], best_gap[1] + 1)
        best_idx = best_gap[0] + np.argmax(ranges[gap_slice])
        target_angle = angles[best_idx]

        # Step 5: Compute steering and speed
        steering = np.clip(steer_gain * target_angle, -0.5, 0.5)
        speed = max_speed - (max_speed - min_speed) * abs(steering) / 0.5
        speed = max(min_speed, min(max_speed, speed))

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GapFollowerNode())
    rclpy.shutdown()
```

---

## 14. Map-Based Navigation: Trajectory Tracking

### 14.1 Pure Pursuit controller

```python
# File: covapsy_ws/src/covapsy_nav/covapsy_nav/pure_pursuit_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import tf_transformations

class PurePursuitNode(Node):
    """Pure Pursuit trajectory tracking for map-based racing mode."""

    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('wheelbase', 0.257)    # TT-02 wheelbase in meters
        self.declare_parameter('lookahead_min', 0.5)   # meters
        self.declare_parameter('lookahead_max', 1.5)    # meters
        self.declare_parameter('speed_factor', 0.3)     # lookahead = speed * factor
        self.declare_parameter('max_speed', 2.5)        # m/s
        self.declare_parameter('max_steering', 0.4)     # radians

        self.path_sub = self.create_subscription(Path, '/racing_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pursuit', 10)

        self.path = None
        self.current_pose = None
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.path is None or self.current_pose is None or len(self.path) < 2:
            return

        L = self.get_parameter('wheelbase').value
        ld_min = self.get_parameter('lookahead_min').value
        ld_max = self.get_parameter('lookahead_max').value
        speed_factor = self.get_parameter('speed_factor').value
        max_speed = self.get_parameter('max_speed').value
        max_steer = self.get_parameter('max_steering').value

        # Current state
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        q = self.current_pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Adaptive lookahead based on current speed (approximated)
        ld = np.clip(max_speed * speed_factor, ld_min, ld_max)

        # Find lookahead point on path
        path_arr = np.array(self.path)
        dists = np.hypot(path_arr[:, 0] - px, path_arr[:, 1] - py)
        nearest_idx = np.argmin(dists)

        target = None
        for i in range(nearest_idx, len(self.path)):
            if dists[i] >= ld:
                target = self.path[i]
                break
        if target is None:
            target = self.path[-1]

        # Transform target to vehicle frame
        dx = target[0] - px
        dy = target[1] - py
        local_x = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        local_y = dx * np.sin(-yaw) + dy * np.cos(-yaw)

        # Pure pursuit formula
        if abs(local_x**2 + local_y**2) < 0.001:
            return
        curvature = 2.0 * local_y / (local_x**2 + local_y**2)
        steering = np.arctan(L * curvature)
        steering = np.clip(steering, -max_steer, max_steer)

        # Speed: inversely proportional to curvature
        speed = max_speed * (1.0 - 0.7 * abs(steering) / max_steer)

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PurePursuitNode())
    rclpy.shutdown()
```

---

## 15. Dual-Mode State Machine

### 15.1 Mode controller node

```python
# File: covapsy_ws/src/covapsy_nav/covapsy_nav/mode_controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import numpy as np

class ModeControllerNode(Node):
    """
    Top-level state machine selecting between:
      IDLE     → waiting for start signal
      REACTIVE → Follow-the-Gap (unknown track, first lap, or SLAM failure)
      MAPPING  → Slow driving while SLAM builds map (setup laps)
      RACING   → Pure Pursuit on optimized trajectory (after map acquired)
      STOPPED  → Remote stop received or emergency
    """

    def __init__(self):
        super().__init__('mode_controller')

        # State
        self.mode = 'IDLE'

        # Subscriptions
        self.reactive_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_reactive', self.reactive_cb, 10)
        self.pursuit_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_pursuit', self.pursuit_cb, 10)
        self.start_sub = self.create_subscription(
            Bool, '/race_start', self.start_cb, 10)
        self.stop_sub = self.create_subscription(
            Bool, '/race_stop', self.stop_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/car_mode', 10)

        # Store latest commands
        self.reactive_cmd = Twist()
        self.pursuit_cmd = Twist()
        self.has_map = False
        self.lap_count = 0

        self.create_timer(0.02, self.control_loop)  # 50 Hz

    def start_cb(self, msg):
        if msg.data:
            self.mode = 'REACTIVE'  # Always start in reactive mode
            self.get_logger().info('Race started → REACTIVE mode')

    def stop_cb(self, msg):
        if msg.data:
            self.mode = 'STOPPED'
            self.get_logger().info('Race stopped')

    def reactive_cb(self, msg):
        self.reactive_cmd = msg

    def pursuit_cb(self, msg):
        self.pursuit_cmd = msg

    def scan_cb(self, msg):
        pass  # Could use for lap counting via start-line detection

    def set_racing_mode(self):
        """Call this when map is ready and localization is stable."""
        self.has_map = True
        self.mode = 'RACING'
        self.get_logger().info('Map acquired → RACING mode')

    def control_loop(self):
        cmd = Twist()

        if self.mode == 'IDLE' or self.mode == 'STOPPED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.mode == 'REACTIVE' or self.mode == 'MAPPING':
            cmd = self.reactive_cmd
            if self.mode == 'MAPPING':
                # Cap speed during mapping
                cmd.linear.x = min(cmd.linear.x, 0.5)
        elif self.mode == 'RACING':
            # Use pursuit if available, fallback to reactive
            if abs(self.pursuit_cmd.linear.x) > 0.01:
                cmd = self.pursuit_cmd
            else:
                cmd = self.reactive_cmd

        self.cmd_pub.publish(cmd)

        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ModeControllerNode())
    rclpy.shutdown()
```

---

## 16. Trackside Computer Setup

### 16.1 Purpose and 2026 rules

COVAPSY 2026 allows an off-car computer that cannot be touched after the start signal. Use it for:
- Heavy SLAM map optimization
- RViz2 visualization and monitoring
- Map-to-trajectory optimization
- ROS2 bag recording
- Neural network inference if needed

**Safety rule:** The car must remain safe if WiFi drops. All reactive control and safety watchdogs run on-car.

### 16.2 Network setup

```bash
# On the trackside laptop:
# 1. Connect to the same WiFi network as the Pi 5
# 2. Set the same ROS_DOMAIN_ID

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash

# Verify connectivity:
ros2 topic list  # Should show topics from the car

# Launch visualization:
ros2 run rviz2 rviz2 -d ~/covapsy_rviz_config.rviz

# Record bags:
ros2 bag record /scan /scan_filtered /odom /cmd_vel /car_mode /map -o race_bag
```

### 16.3 CycloneDDS multicast configuration

Both Pi and laptop need matching CycloneDDS config. Create on both:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

---

## 17. ROS2 Workspace Structure

### 17.1 Directory layout

```
~/covapsy_ws/
├── src/
│   ├── covapsy_bringup/        # Launch files, config, URDF
│   │   ├── launch/
│   │   │   ├── car_full.launch.py       # Complete car bringup
│   │   │   ├── lidar.launch.py          # LiDAR only
│   │   │   ├── realsense.launch.py      # Camera only
│   │   │   ├── slam.launch.py           # SLAM toolbox
│   │   │   └── nav.launch.py            # Navigation (reactive + pursuit)
│   │   ├── config/
│   │   │   ├── slam_toolbox_params.yaml
│   │   │   ├── nav2_params.yaml
│   │   │   └── car_params.yaml
│   │   ├── urdf/
│   │   │   └── covapsy_car.urdf.xacro
│   │   └── maps/                        # Saved maps go here
│   │
│   ├── covapsy_bridge/         # STM32 HAT communication
│   │   └── covapsy_bridge/
│   │       ├── stm32_bridge_node.py
│   │       └── tft_status_node.py
│   │
│   ├── covapsy_perception/     # Sensor processing
│   │   └── covapsy_perception/
│   │       ├── scan_filter_node.py
│   │       └── border_detect_node.py
│   │
│   ├── covapsy_nav/            # Planning and control
│   │   └── covapsy_nav/
│   │       ├── gap_follower_node.py
│   │       ├── pure_pursuit_node.py
│   │       └── mode_controller_node.py
│   │
│   └── realsense-ros/          # Cloned RealSense ROS2 wrapper (if building from source)
│
├── install/
├── build/
└── log/
```

### 17.2 Build commands

```bash
cd ~/covapsy_ws
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build (limit parallel jobs for 2GB RAM)
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## 18. Launch Files

### 18.1 Full car bringup

```python
# File: covapsy_ws/src/covapsy_bringup/launch/car_full.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('covapsy_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='reactive',
            description='Start mode: reactive, mapping, or racing'),

        # STM32 bridge
        Node(package='covapsy_bridge', executable='stm32_bridge_node',
             name='stm32_bridge', output='screen'),

        # TFT status display
        Node(package='covapsy_bridge', executable='tft_status_node',
             name='tft_status', output='screen'),

        # micro-ROS agent (alternative to custom bridge if using micro-ROS)
        # Node(package='micro_ros_agent', executable='micro_ros_agent',
        #      arguments=['serial', '--dev', '/dev/ttyAMA0', '-b', '115200']),

        # LiDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'lidar.launch.py'))),

        # Scan filter
        Node(package='covapsy_perception', executable='scan_filter_node',
             name='scan_filter', output='screen',
             parameters=[{'max_range': 5.0, 'min_range': 0.15, 'median_window': 3}]),

        # Gap follower (reactive navigation, always running as fallback)
        Node(package='covapsy_nav', executable='gap_follower_node',
             name='gap_follower', output='screen',
             parameters=[{
                 'car_width': 0.30,
                 'max_speed': 2.0,
                 'min_speed': 0.5,
                 'safety_radius': 0.20,
                 'disparity_threshold': 0.5,
             }]),

        # Pure pursuit (map-based, publishes only when path is available)
        Node(package='covapsy_nav', executable='pure_pursuit_node',
             name='pure_pursuit', output='screen',
             parameters=[{
                 'wheelbase': 0.257,
                 'lookahead_min': 0.5,
                 'lookahead_max': 1.5,
                 'max_speed': 2.5,
             }]),

        # Mode controller (top-level state machine)
        Node(package='covapsy_nav', executable='mode_controller_node',
             name='mode_controller', output='screen'),

        # Static transforms
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.10', '0', '0.12', '0', '0.1', '0', 'base_link', 'camera_link']),
    ])
```

---

## 19. Parameter Tuning Guide

### 19.1 Parameters that matter most (tune in this order)

| Parameter | Node | Start value | Range | How to tune |
|-----------|------|-------------|-------|-------------|
| `safety_radius` | gap_follower | 0.20 m | 0.10–0.35 | Increase if car clips walls. Decrease for tighter tracks |
| `max_speed` | gap_follower | 2.0 m/s | 0.5–3.0 | Start at 0.5, increase by 0.3 per test session |
| `disparity_threshold` | gap_follower | 0.5 m | 0.2–1.0 | Lower = more conservative gap detection |
| `car_width` | gap_follower | 0.30 m | 0.25–0.35 | Measure your car with all sensors mounted |
| `lookahead_min` | pure_pursuit | 0.5 m | 0.3–1.0 | Lower = more responsive, higher = smoother |
| `max_laser_range` | slam_toolbox | 5.0 m | 3.0–8.0 | Lower = faster SLAM, higher = better map |
| `solver_threads` | slam_toolbox | 2 | 1–3 | Monitor CPU with htop; leave ≥1 core free |

### 19.2 Speed ladder protocol

Do NOT jump to high speed. Follow this progression:

```
Session 1: max_speed = 0.5 m/s → Verify gap follower works, no wall contact
Session 2: max_speed = 1.0 m/s → Tune safety_radius and disparity_threshold
Session 3: max_speed = 1.5 m/s → Verify stability in tight turns
Session 4: max_speed = 2.0 m/s → Competition baseline speed
Session 5: max_speed = 2.5 m/s → Only after 20 consecutive clean laps at 2.0
Session 6: max_speed = 3.0 m/s → Only for time trials with known track
```

---

## 20. Testing and Validation Protocol

### 20.1 Unit tests (run in simulation)

| Test | Tool | Pass criteria |
|------|------|---------------|
| Scan filter | F1TENTH Gym | Filtered scan has no NaN/Inf, range within bounds |
| Gap follower | F1TENTH Gym | 10 clean laps on 3 different tracks at 2.0 m/s |
| Pure pursuit | F1TENTH Gym | Follows recorded path with <5cm cross-track error |
| Border detection | Recorded RealSense bag | Correctly identifies green/red in 90%+ of frames |
| STM32 bridge | minicom + loopback | Commands sent match commands received, watchdog triggers at 250ms |

### 20.2 Integration tests (on real car)

| Test | Pass criteria |
|------|---------------|
| Thermal stability | 10-minute full-stack run, temp stays below 80°C |
| Power stability | Full battery discharge cycle, no brownout or reboot |
| Command-loss failsafe | Disconnect Pi UART: STM32 must stop car within 250ms |
| Remote start/stop | Start/stop commands work reliably from competition remote |
| Straight + turn (homologation) | Car navigates straight and 90° turn without touching borders |
| Obstacle avoidance (homologation) | Car avoids a car-sized static obstacle placed on track |
| Reverse when blocked (homologation) | Car reverses and reroutes when facing a dead end |
| 20 consecutive clean laps | No border contact at competition speed |

### 20.3 ROS bag recording for debugging

```bash
# Always record during real-car tests
ros2 bag record -a -o test_run_$(date +%Y%m%d_%H%M%S) --max-cache-size 100000000

# Replay and analyze on laptop
ros2 bag play test_run_20260401_143000/
ros2 run rviz2 rviz2  # Visualize the replay
```

---

## 21. Race-Day Checklist

### 21.1 Before leaving for competition

- [ ] Flash a backup microSD with the identical OS image
- [ ] Charge 3 NiMH batteries to full
- [ ] Pack: Pi 5, STM32 HAT, RPLidar, RealSense, cables, active cooler, buck converter, spare microSD, USB keyboard, HDMI cable, laptop, router (if allowed)
- [ ] Verify all code compiles cleanly with `colcon build`
- [ ] Run 20 consecutive clean laps in lab

### 21.2 Pit setup (before first race)

- [ ] Assemble car, verify all mechanical fasteners tight (vibration loosens screws)
- [ ] Power on Pi, SSH in, verify `ros2 topic list` shows all expected topics
- [ ] Verify LiDAR spinning and `/scan` publishing at 10 Hz
- [ ] Verify RealSense streaming (if using): `ros2 topic hz /camera/camera/color/image_raw`
- [ ] Verify STM32 communication: `ros2 topic echo /wheel_speed`
- [ ] Verify remote start/stop works
- [ ] Run `vcgencmd measure_temp` — should be <60°C idle
- [ ] Connect trackside laptop, verify `ros2 topic list` shows car topics

### 21.3 Homologation

- [ ] Demonstrate straight + turn driving
- [ ] Demonstrate obstacle avoidance
- [ ] Demonstrate reverse behavior
- [ ] Demonstrate remote stop kills the car immediately

### 21.4 During competition

- [ ] Time trial: Use RACING mode (Pure Pursuit) if map available, REACTIVE (gap follower) if not
- [ ] Multi-car race: Switch to REACTIVE mode with `max_speed` reduced 20% for safety margins
- [ ] Between races: Check battery voltage, swap if below 7.0V. Check temperature. Review any failures from ROS bags.

### 21.5 Priority order (what to optimize and when to stop)

```
1. FINISH THE RACE (reliability > speed, always)
2. Finish with no border contacts (clean laps score better)
3. Finish faster (tune speed only after 1 and 2 are solid)
4. Overtake opponents (only if car is stable and you understand the traffic)
```

---

## APPENDIX: Key Reference Repositories

| Repository | URL | What to study |
|-----------|-----|---------------|
| Official COVAPSY | `github.com/ajuton-ens/CourseVoituresAutonomesSaclay` | Simulator, hardware designs, rules |
| Sorbonne 2025 | `github.com/SU-Bolides/Course_2025` | Full ROS stack, STM32 code, Webots sim |
| Sorbonne 2024 | `github.com/SU-Bolides/Course_2024` | Reactive vs SLAM dual-car comparison |
| INTech RL approach | `github.com/Association-INTech/CoVAPSy` | Reinforcement learning in Webots |
| ECE Paris-Saclay | `github.com/Surensemble/ECE-Course-de-voitures-autonomes-Paris-Saclay` | Electronics + simulation |
| F1TENTH Gym ROS | `github.com/f1tenth/f1tenth_gym_ros` | Fast LiDAR-based racing sim for ROS2 |
| ForzaETH race stack | `github.com/ForzaETH/race_stack` | State-of-the-art autonomous racing |
| TUM trajectory opt | `github.com/TUMFTM/global_racetrajectory_optimization` | Minimum curvature raceline |

---

*End of implementation plan. This document defines one complete path from bare hardware to race-ready autonomy. Every command, file path, parameter, and test criterion is specified. An AI agent should execute sections 2–18 in order, then run the validation protocol in section 20 before any speed optimization in section 19.*