#!/bin/bash
# ============================================================================
# COVAPSY ROS2 Jazzy Installation Script
# Run on Pi 5 after setup_pi.sh
# Usage: bash install_ros2.sh
# ============================================================================

set -e

echo "=========================================="
echo "  COVAPSY ROS2 Jazzy Installation"
echo "=========================================="

# -- Add ROS2 repository --
echo "[1/5] Adding ROS2 apt repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# -- Install ROS2 base --
echo "[2/5] Installing ROS2 Jazzy base..."
sudo apt install -y ros-jazzy-ros-base

# -- Install competition packages --
echo "[3/5] Installing competition ROS2 packages..."
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rplidar-ros \
  ros-jazzy-realsense2-camera \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-micro-ros-agent \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf-transformations \
  ros-jazzy-robot-localization \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-ackermann-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-serial \
  python3-colcon-common-extensions \
  python3-rosdep

# Initialize rosdep
echo "[4/5] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# -- Configure shell environment --
echo "[5/5] Configuring shell environment..."
BASHRC="$HOME/.bashrc"

if ! grep -q "COVAPSY ROS2" "$BASHRC"; then
    cat >> "$BASHRC" << 'EOF'

# === COVAPSY ROS2 Environment ===
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42

# CycloneDDS config
if [ -f "$HOME/covapsy_ws/src/covapsy_bringup/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/covapsy_ws/src/covapsy_bringup/config/cyclonedds.xml
fi

# Workspace
if [ -f "$HOME/covapsy_ws/install/setup.bash" ]; then
    source "$HOME/covapsy_ws/install/setup.bash"
fi
EOF
    echo "  Shell environment configured"
fi

echo ""
echo "=========================================="
echo "  ROS2 Jazzy installation complete!"
echo "  Next: copy ros2_ws to ~/covapsy_ws"
echo "  Then: cd ~/covapsy_ws && colcon build --symlink-install --parallel-workers 2"
echo "=========================================="
