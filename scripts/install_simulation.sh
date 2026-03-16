#!/bin/bash
# ============================================================================
# COVAPSY self-contained simulation helper
# Usage: bash install_simulation.sh
# ============================================================================

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_DIR/simulation/webots"
WORLD_ROS2="$SIM_DIR/worlds/Piste_CoVAPSy_2025a_ros2.wbt"
WORLD_STANDALONE="$SIM_DIR/worlds/Piste_CoVAPSy_2025a_standalone.wbt"

echo "=========================================="
echo "  COVAPSY Webots Simulation Setup"
echo "=========================================="

if [ ! -d "$SIM_DIR" ]; then
  echo "ERROR: simulation directory not found: $SIM_DIR"
  exit 1
fi

if [ ! -f "$WORLD_ROS2" ]; then
  echo "ERROR: ROS2 world missing: $WORLD_ROS2"
  exit 1
fi

echo "Simulation assets are already self-contained in:"
echo "  $SIM_DIR"
echo ""
echo "Use one of these worlds:"
echo "  ROS2 bridge mode:    $WORLD_ROS2"
echo "  Standalone mode:     $WORLD_STANDALONE"
echo ""
echo "ROS2 bridge controller:"
echo "  $SIM_DIR/controllers/covapsy_ros2_bridge/covapsy_ros2_bridge.py"
echo ""
echo "Quickstart (ROS2 bridge mode):"
echo "  1. source /opt/ros/jazzy/setup.bash"
echo "  2. cd $PROJECT_DIR/ros2_ws && colcon build --symlink-install"
echo "  3. source install/setup.bash"
echo "  4. ros2 launch covapsy_bringup sim_webots.launch.py"
echo "  5. Optional: mode:=reactive|learning|race, world:=<world_file>, headless:=true"
echo "=========================================="
