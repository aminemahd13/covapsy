#!/bin/bash
# ============================================================================
# COVAPSY Pi 5 Setup Script
# Run this on the Raspberry Pi 5 after flashing Ubuntu Server 24.04 LTS
# Usage: bash setup_pi.sh
# ============================================================================

set -e

echo "=========================================="
echo "  COVAPSY Pi 5 Initial Setup"
echo "=========================================="

# -- System Update --
echo "[1/7] Updating system..."
sudo apt update && sudo apt full-upgrade -y

# -- Disable unnecessary services --
echo "[2/7] Disabling unnecessary services..."
sudo systemctl disable snapd snapd.socket snapd.seeded 2>/dev/null || true
sudo systemctl disable unattended-upgrades 2>/dev/null || true
sudo systemctl disable ModemManager 2>/dev/null || true
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# -- Install essential tools --
echo "[3/7] Installing essential tools..."
sudo apt install -y htop tmux git build-essential cmake python3-pip \
  python3-venv i2c-tools minicom screen

# -- Swap configuration (essential for 2GB RAM) --
echo "[4/7] Configuring swap..."
if [ ! -f /swapfile ]; then
    sudo fallocate -l 4G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    echo "  Swap file created (4GB)"
else
    echo "  Swap file already exists"
fi

# ZRAM
sudo apt install -y zram-tools
echo -e "ALGO=lz4\nPERCENT=50" | sudo tee /etc/default/zramswap
sudo systemctl enable zramswap
sudo systemctl restart zramswap

# -- udev rules --
echo "[5/7] Creating udev rules..."

# RPLidar
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="rplidar", MODE="0666"
EOF

# STM32 USB serial symlink (/dev/stm32_mcu)
sudo tee /etc/udev/rules.d/99-stm32-mcu.rules << 'EOF'
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="0483", SYMLINK+="stm32_mcu", MODE="0666"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ENV{ID_VENDOR_ID}=="0483", SYMLINK+="stm32_mcu", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# -- Boot configuration --
echo "[6/7] Configuring boot parameters..."
BOOT_CONFIG="/boot/firmware/config.txt"
if [ -f "$BOOT_CONFIG" ]; then
    if ! grep -q "COVAPSY" "$BOOT_CONFIG"; then
        sudo tee -a "$BOOT_CONFIG" << 'EOF'

# === COVAPSY Pi 5 Configuration ===
dtparam=uart0=on
usb_max_current_enable=1
dtparam=i2c_arm=on
EOF
        echo "  Boot config updated"
    else
        echo "  Boot config already configured"
    fi
fi

# -- Python dependencies --
echo "[7/7] Installing Python dependencies..."
pip3 install --user pyserial numpy rpi-hardware-pwm

echo ""
echo "=========================================="
echo "  Pi 5 setup complete!"
echo "  Next: run install_ros2.sh"
echo "=========================================="
