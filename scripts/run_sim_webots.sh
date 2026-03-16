#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_DIR/ros2_ws"

if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "ERROR: Missing $ROS_WS/install/setup.bash"
  echo "Build first:"
  echo "  cd $ROS_WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"

cd "$ROS_WS"
exec ros2 launch covapsy_bringup sim_webots.launch.py "$@"
