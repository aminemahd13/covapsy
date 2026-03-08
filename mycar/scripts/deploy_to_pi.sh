#!/bin/bash
# ============================================================================
# Deploy COVAPSY workspace to Pi 5
# Usage: bash deploy_to_pi.sh [PI_IP]
# Example: bash deploy_to_pi.sh 192.168.1.42
# ============================================================================

PI_IP="${1:-covapsy-car.local}"
PI_USER="covapsy"
WORKSPACE_SRC="$(dirname "$(dirname "$(readlink -f "$0")")")/ros2_ws/src"

echo "Deploying to ${PI_USER}@${PI_IP}..."
echo "Source: ${WORKSPACE_SRC}"

# Sync ROS2 packages to Pi
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='__pycache__' --exclude='.git' \
  "${WORKSPACE_SRC}/" \
  "${PI_USER}@${PI_IP}:~/covapsy_ws/src/"

echo ""
echo "Deployed! Now SSH into the Pi and build:"
echo "  ssh ${PI_USER}@${PI_IP}"
echo "  cd ~/covapsy_ws"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build --symlink-install --parallel-workers 2"
