#!/usr/bin/env bash
# Attach Gazebo GUI to an already-running headless server.
# Start the sim first:  ros2 launch rover_simulation demo.launch.py mode:=creep

set -e
source /opt/ros/jazzy/setup.bash
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
  WS="$WS_DIR"
else
  WS=~/lunar-rover-quantum
fi
source "$WS/install/setup.bash"
PKG="$WS/install/rover_simulation/share/rover_simulation"
export GZ_SIM_RESOURCE_PATH="$PKG/worlds:$PKG/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib

echo "Opening Gazebo GUI (connects to running server)..."
echo "If this crashes on WSL, use RViz instead:"
echo "  ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true"
exec gz sim -g -v3
